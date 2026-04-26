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

    // 1. If a previous thread is somehow still finishing up, join it safely
    if (m_workerThread.joinable()) {
        m_workerThread.join();
    }

    // Disable the UI
    setEnabled(false);
    m_statusLabel->setText("Decimating mesh. Please wait...");

    // 2. Make a safe, local copy of the model data on the main thread 
    auto modelDataCopy = m_currentModel;

    // 3. Fire off the background thread (assigned to our member variable)
    m_workerThread = std::thread([this, modelDataCopy, targetVertexCount]() {

        // Build the OpenMesh (using the safe copy)
        MeshType optimisedMesh = MeshProcessor::convertRawToOpenMesh(modelDataCopy);

        // Decimate the mesh
        MeshProcessor::decimateMesh(optimisedMesh, targetVertexCount);

        // Extract the optimised vertices and indices into standard local variables
        std::vector<Vertex> decimatedVertices;
        std::vector<unsigned int> decimatedIndices;
        MeshProcessor::extractRawFromOpenMesh(optimisedMesh, decimatedVertices, decimatedIndices);

        // 4: Schedule viewport update back onto the main UI thread.
        QMetaObject::invokeMethod(this, [this,
            v = std::move(decimatedVertices),
            i = std::move(decimatedIndices)]() mutable {

                m_viewport->UpdateMesh(v, i);
                m_viewport->update();

                setEnabled(true);
                m_statusLabel->setText(QString("Decimation Complete. Vertices: %1, Triangles: %2")
                    .arg(v.size())
                    .arg(i.size() / 3));
            });

        });
}