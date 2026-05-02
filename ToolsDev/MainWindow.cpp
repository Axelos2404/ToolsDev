#include "MainWindow.h"
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QPushButton>
#include <CoMISo/Solver/CholmodSolver.hh>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("ToolsDev");
    resize(1280, 720);

    // Viewport
    m_viewport = new ViewportWidget(this);
    setCentralWidget(m_viewport);

    // Menu bar
    QMenu* fileMenu = menuBar()->addMenu("&File");
    fileMenu->addAction("&Open Model...", QKeySequence::Open, this, &MainWindow::OnFileOpen);
    fileMenu->addSeparator();
    fileMenu->addAction("E&xit", QKeySequence::Quit, this, &QMainWindow::close);

    QMenu* helpMenu = menuBar()->addMenu("&Help");
    helpMenu->addAction("&About", this, [this]() {
        QMessageBox::about(this, "About ToolsDev",
            "ToolsDev - Auto Retopology & UV Unwrapping Tool");
        });

    // Tools Dock Widget
    QDockWidget* toolsDock = new QDockWidget("Retopology Tools", this);
    toolsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* dockContents = new QWidget();
    QVBoxLayout* dockLayout = new QVBoxLayout(dockContents);

    // Slider setup
    QLabel* targetVertLabel = new QLabel("Target Vertex Count: 1000");
    QSlider* targetVertSlider = new QSlider(Qt::Horizontal);
    targetVertSlider->setRange(10, 100000); // Adjust maximum as needed by your models
    targetVertSlider->setValue(1000);

    // Update label when slider moves
    connect(targetVertSlider, &QSlider::valueChanged, targetVertLabel, [targetVertLabel](int value) {
        targetVertLabel->setText(QString("Target Vertex Count: %1").arg(value));
        });

    QPushButton* applyDecimationBtn = new QPushButton("Apply Decimation");

    // Connect the button to your function, passing the slider value
    connect(applyDecimationBtn, &QPushButton::clicked, this, [this, targetVertSlider]() {
        OnApplyDecimationClicked(targetVertSlider->value());
        });

    dockLayout->addWidget(targetVertLabel);
    dockLayout->addWidget(targetVertSlider);
    dockLayout->addWidget(applyDecimationBtn);
    dockLayout->addStretch();

    toolsDock->setWidget(dockContents);
    addDockWidget(Qt::RightDockWidgetArea, toolsDock);

    // Status bar
    m_statusLabel = new QLabel("Ready");
    statusBar()->addWidget(m_statusLabel);
}

MainWindow::~MainWindow()
{
    if (m_workerThread.joinable())
        m_workerThread.join();
}

void MainWindow::OnFileOpen()
{
    QString filePath = QFileDialog::getOpenFileName(this,
        "Open 3D Model",
        QString(),
        "3D Models (*.obj *.fbx *.gltf *.glb *.dae *.stl *.ply *.3ds);;All Files (*.*)");

    if (filePath.isEmpty())
        return;

    if (m_loader.Load(filePath.toStdString(), m_currentModel))
    {
        m_viewport->SetModel(m_currentModel);

        int totalVerts = 0, totalTris = 0;
        for (const auto& mesh : m_currentModel.meshes)
        {
            totalVerts += static_cast<int>(mesh.vertices.size());
            totalTris += static_cast<int>(mesh.indices.size()) / 3;
        }

        m_statusLabel->setText(QString("Loaded: %1 meshes, %2 vertices, %3 triangles")
            .arg(m_currentModel.meshes.size())
            .arg(totalVerts)
            .arg(totalTris));
    }
    else
    {
        QMessageBox::warning(this, "Load Error",
            QString::fromStdString(m_loader.GetLastError()));
    }
}

void MainWindow::OnApplyDecimationClicked(int targetVertexCount)
{
    if (m_currentModel.meshes.empty())
    {
        QMessageBox::information(this, "No Model", "Please load a model before applying decimation.");
        return;
    }

    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }

    setEnabled(false);
    m_statusLabel->setText("Processing sub-meshes individually. Please wait...");

    auto modelDataCopy = m_currentModel;

    m_workerThread = std::thread([this, modelDataCopy, targetVertexCount]() {

        std::vector<Vertex> finalVertices;
        std::vector<unsigned int> finalIndices;

        // Calculate total original vertices to distribute the target budget proportionally
        size_t totalOriginalVerts = 0;
        for (const auto& mesh : modelDataCopy.meshes) {
            totalOriginalVerts += mesh.vertices.size();
        }

        bool anyRetopoFailed = false;
        double gridDensity = 30.0;

        // Iterate through each sub-mesh (tires, chassis, windows) independently
        for (size_t m_idx = 0; m_idx < modelDataCopy.meshes.size(); ++m_idx)
        {
            const auto& rawMesh = modelDataCopy.meshes[m_idx];
            if (rawMesh.vertices.empty() || rawMesh.indices.empty()) continue;

            // 1. Calculate proportional target vertex count for this specific part
            double ratio = static_cast<double>(rawMesh.vertices.size()) / static_cast<double>(totalOriginalVerts);
            int localTarget = std::max(10, static_cast<int>(targetVertexCount * ratio));

            // 2. Package into a temporary ModelData for the converter
            ModelData singleModel;
            singleModel.meshes.push_back(rawMesh);

            // 3. Convert and Decimate just this piece
            MeshType optMesh = MeshProcessor::convertRawToOpenMesh(singleModel);
            MeshProcessor::decimateMesh(optMesh, localTarget);

            // 4. Extract
            std::vector<Vertex> decVerts;
            std::vector<unsigned int> decInds;
            MeshProcessor::extractRawFromOpenMesh(optMesh, decVerts, decInds);

            // 5. Retopologize (ONLY if the mesh is large enough to survive MIQ)
            std::vector<Vertex> quadVerts;
            std::vector<unsigned int> quadInds;
            bool retopoOk = false;

            // SAFEGUARD: MIQ will crash on tiny, degenerate pieces. 
            // If the piece has fewer than 100 vertices, skip MIQ entirely.
            if (decVerts.size() > 100 && (decInds.size() / 3) > 50)
            {
                retopoOk = RetopoProcessor::processRetopology(decVerts, decInds, quadVerts, quadInds, gridDensity);
            }
            else
            {
                OutputDebugStringA(("[Pipeline] Skipping MIQ for tiny mesh part (" + std::to_string(decVerts.size()) + " verts)\n").c_str());
            }

            // If a single tiny part fails MIQ (or was skipped), fall back to its decimated triangle version
            if (!retopoOk) {
                anyRetopoFailed = true;
                quadVerts = decVerts;
                quadInds = decInds;
            }

            // 6. Accumulate into the final global buffers for the viewport
            unsigned int vertexOffset = static_cast<unsigned int>(finalVertices.size());
            finalVertices.insert(finalVertices.end(), quadVerts.begin(), quadVerts.end());

            for (unsigned int idx : quadInds) {
                finalIndices.push_back(idx + vertexOffset);
            }

            OutputDebugStringA(("[Pipeline] Processed mesh " + std::to_string(m_idx + 1) + "/" + std::to_string(modelDataCopy.meshes.size()) + "\n").c_str());
        }

        // 7. Schedule viewport update back onto the main UI thread.
        QMetaObject::invokeMethod(this, [this,
            v = std::move(finalVertices),
            i = std::move(finalIndices),
            anyRetopoFailed]() mutable {

                m_viewport->UpdateMesh(v, i);
                m_viewport->update();

                setEnabled(true);
                if (!anyRetopoFailed)
                {
                    m_statusLabel->setText(QString("Retopology Complete. Vertices: %1, Triangles: %2")
                        .arg(v.size())
                        .arg(i.size() / 3));
                }
                else
                {
                    m_statusLabel->setText(QString("Finished (Some tiny parts fell back to Triangles). Vertices: %1, Triangles: %2")
                        .arg(v.size())
                        .arg(i.size() / 3));
                }
            });

        });
}