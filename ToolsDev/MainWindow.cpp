#define NOMINMAX
#include <Windows.h>

#include "MainWindow.h"
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDockWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <fstream>
#include <filesystem>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("ToolsDev");
    resize(1280, 720);

    m_viewport = new ViewportWidget(this);
    setCentralWidget(m_viewport);

    QMenu* fileMenu = menuBar()->addMenu("&File");
    fileMenu->addAction("&Open Model...", QKeySequence::Open, this, &MainWindow::OnFileOpen);
    fileMenu->addSeparator();
    fileMenu->addAction("E&xit", QKeySequence::Quit, this, &QMainWindow::close);

    QDockWidget* toolsDock = new QDockWidget("Retopology Tools", this);
    toolsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* dockContents = new QWidget();
    QVBoxLayout* dockLayout = new QVBoxLayout(dockContents);

    QLabel* targetVertLabel = new QLabel("Target Vertex Count:");

    // Wire up the new class-level controls
    m_targetVertSlider = new QSlider(Qt::Horizontal);
    m_targetVertSlider->setRange(10, 200000);
    m_targetVertSlider->setValue(100000);

    m_targetVertSpinBox = new QSpinBox();
    m_targetVertSpinBox->setRange(10, 200000);
    m_targetVertSpinBox->setValue(100000);
    m_targetVertSpinBox->setSingleStep(1000);

    connect(m_targetVertSlider, &QSlider::valueChanged, m_targetVertSpinBox, &QSpinBox::setValue);
    connect(m_targetVertSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), m_targetVertSlider, &QSlider::setValue);

    QHBoxLayout* targetLayout = new QHBoxLayout();
    targetLayout->addWidget(targetVertLabel);
    targetLayout->addWidget(m_targetVertSpinBox);

    // Context Checkbox
    m_chkShowContext = new QCheckBox("Show the rest of the model");
    m_chkShowContext->setChecked(true); // Default to showing everything
    connect(m_chkShowContext, &QCheckBox::toggled, this, [this]() {
        RefreshViewportAndUI();
        });

    m_chkUseInstantMeshes = new QCheckBox("Use Quad Retopology (Instant Meshes)");
    m_chkUseInstantMeshes->setChecked(false); // Default to standard safe OpenMesh Decimation

    m_btnPreview = new QPushButton("Preview Result");

    m_btnApply = new QPushButton("Apply (Keep Editing)");
    m_btnApply->setEnabled(false);

    m_btnAccept = new QPushButton("Accept & Next Part");
    m_btnAccept->setEnabled(true);

    connect(m_btnPreview, &QPushButton::clicked, this, [this]() {
        OnApplyDecimationClicked(m_targetVertSpinBox->value());
        });

    connect(m_btnApply, &QPushButton::clicked, this, [this]() {
        OnApplyClicked();
        });

    connect(m_btnAccept, &QPushButton::clicked, this, [this]() {
        OnAcceptClicked();
        });

    // Add to Layout
    dockLayout->addLayout(targetLayout);
    dockLayout->addWidget(m_targetVertSlider);
    dockLayout->addWidget(m_chkShowContext);
    dockLayout->addWidget(m_chkUseInstantMeshes);
    dockLayout->addWidget(m_btnPreview);
    dockLayout->addWidget(m_btnApply);
    dockLayout->addWidget(m_btnAccept);
    dockLayout->addStretch();

    toolsDock->setWidget(dockContents);
    addDockWidget(Qt::RightDockWidgetArea, toolsDock);

    m_statusLabel = new QLabel("Ready");
    statusBar()->addWidget(m_statusLabel);
}

MainWindow::~MainWindow()
{
    if (m_workerThread.joinable()) m_workerThread.join();
}

void MainWindow::OnFileOpen()
{
    QString filePath = QFileDialog::getOpenFileName(this, "Open 3D Model", QString(), "3D Models (*.obj *.fbx *.gltf *.glb *.dae *.stl *.ply *.3ds);;All Files (*.*)");
    if (filePath.isEmpty()) return;

    if (m_loader.Load(filePath.toStdString(), m_currentModel))
    {
        m_currentMeshIndex = 0;
        m_retopoVertices.clear();
        m_retopoIndices.clear();

        m_showingPreview = false;
        m_previewVertices.clear();
        m_previewIndices.clear();

        if (m_btnAccept) m_btnAccept->setEnabled(false);
        if (m_btnPreview) m_btnPreview->setEnabled(true);

        m_viewport->SetModel(m_currentModel);
        RefreshViewportAndUI();
    }
    else
    {
        QMessageBox::warning(this, "Load Error", QString::fromStdString(m_loader.GetLastError()));
    }
}

void ExportToOBJ(const std::string& filepath, const std::vector<Vertex>& verts, const std::vector<unsigned int>& inds)
{
    std::filesystem::path pathObj(filepath);
    if (pathObj.has_parent_path()) std::filesystem::create_directories(pathObj.parent_path());

    std::ofstream file(filepath);
    if (!file.is_open()) return;

    for (const auto& v : verts) file << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2] << "\n";
    for (const auto& v : verts) file << "vt " << v.texCoord[0] << " " << v.texCoord[1] << "\n";
    for (const auto& v : verts) file << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2] << "\n";

    // Attempt to merge back triangles into quads based on the RetopoProcessor generation logic (0,1,2) + (2,3,0)
    for (size_t i = 0; i < inds.size(); i += 3) {

        // Check if there is another triangle after this one that completes the quad format: (0,1,2) + (2,3,0)
        if (i + 5 < inds.size()) {
            unsigned int t1_0 = inds[i];
            unsigned int t1_1 = inds[i + 1];
            unsigned int t1_2 = inds[i + 2];

            unsigned int t2_0 = inds[i + 3];
            unsigned int t2_1 = inds[i + 4];
            unsigned int t2_2 = inds[i + 5];

            // If the next triangle matches the exact signature of a split quad from RetopoProcessor
            if (t1_2 == t2_0 && t1_0 == t2_2) {
                // It's a quad: print it as 4 points and skip the next triangle
                unsigned int i0 = t1_0 + 1, i1 = t1_1 + 1, i2 = t1_2 + 1, i3 = t2_1 + 1;
                file << "f " << i0 << "/" << i0 << "/" << i0 << " "
                    << i1 << "/" << i1 << "/" << i1 << " "
                    << i2 << "/" << i2 << "/" << i2 << " "
                    << i3 << "/" << i3 << "/" << i3 << "\n";
                i += 3; // Skip next triangle
                continue;
            }
        }

        // Output as standard triangle if it didn't match the quad pattern
        unsigned int i0 = inds[i] + 1, i1 = inds[i + 1] + 1, i2 = inds[i + 2] + 1;
        file << "f " << i0 << "/" << i0 << "/" << i0 << " "
            << i1 << "/" << i1 << "/" << i1 << " "
            << i2 << "/" << i2 << "/" << i2 << "\n";
    }
    file.close();
}

void MainWindow::RefreshViewportAndUI()
{
    if (m_currentModel.meshes.empty()) return;

    std::vector<MeshData> displayMeshes;
    bool showContext = m_chkShowContext && m_chkShowContext->isChecked();

    if (m_currentMeshIndex < m_currentModel.meshes.size())
    {
        const auto& currentPart = m_currentModel.meshes[m_currentMeshIndex];

        if (m_showingPreview) {
            m_statusLabel->setText(QString("PREVIEWING: Part %1 of %2 | '%3' | Preview Vertices: %4")
                .arg(m_currentMeshIndex + 1).arg(m_currentModel.meshes.size()).arg(currentPart.name.c_str()).arg(m_previewVertices.size()));
        }
        else {
            if (m_targetVertSpinBox) {
                m_targetVertSpinBox->setValue(static_cast<int>(currentPart.vertices.size()));
            }
            m_statusLabel->setText(QString("READY: Part %1 of %2 | '%3' | Original Vertices: %4")
                .arg(m_currentMeshIndex + 1).arg(m_currentModel.meshes.size()).arg(currentPart.name.c_str()).arg(currentPart.vertices.size()));
        }

        // Finished parts (Only show if context checkbox is true)
        if (showContext) {
            for (size_t i = 0; i < m_currentMeshIndex; ++i) {
                displayMeshes.push_back(m_currentModel.meshes[i]);
            }
        }

        // The active target part (Always show this!)
        MeshData activePart;
        activePart.vertices = m_showingPreview ? m_previewVertices : currentPart.vertices;
        activePart.indices = m_showingPreview ? m_previewIndices : currentPart.indices;
        displayMeshes.push_back(activePart);

        // The raw future parts (Only show if context checkbox is true)
        if (showContext) {
            for (size_t i = m_currentMeshIndex + 1; i < m_currentModel.meshes.size(); ++i) {
                displayMeshes.push_back(m_currentModel.meshes[i]);
            }
        }

        m_viewport->UpdateMeshes(displayMeshes);
    }
    else
    {
        // Everything finished
        MeshData finalBlob;
        finalBlob.vertices = m_retopoVertices;
        finalBlob.indices = m_retopoIndices;
        displayMeshes.push_back(finalBlob);

        m_statusLabel->setText("All parts processed! Exporting to ..\\Export\\Final_StepByStep.obj");
        ExportToOBJ("..\\Export\\Final_StepByStep.obj", m_retopoVertices, m_retopoIndices);

        if (m_btnPreview) m_btnPreview->setEnabled(false);
        if (m_btnAccept) m_btnAccept->setEnabled(false);

        m_viewport->UpdateMeshes(displayMeshes);
    }
}

void MainWindow::OnApplyDecimationClicked(int targetVertexCount)
{
    if (m_currentMeshIndex >= m_currentModel.meshes.size()) return;
    if (m_workerThread.joinable()) m_workerThread.join();

    setEnabled(false);
    m_statusLabel->setText(QString("Calculating Preview for Part %1...").arg(m_currentMeshIndex + 1));

    auto rawMesh = m_currentModel.meshes[m_currentMeshIndex];

    // Read UI state before firing thread
    bool useRetopology = m_chkUseInstantMeshes->isChecked();

    m_workerThread = std::thread([this, rawMesh, targetVertexCount, useRetopology]() {

        std::vector<Vertex> previewVerts;
        std::vector<unsigned int> previewInds;
        bool ok = false;

        if (rawMesh.vertices.size() > 50 && (rawMesh.indices.size() / 3) > 20) {
            std::vector<Vertex> sourceVerts = rawMesh.vertices;
            std::vector<unsigned int> sourceInds = rawMesh.indices;

            // If the user wants standard fast Decimation OR if the target is too dense for Instant Meshes
            int maxRetopoAllowed = static_cast<int>(rawMesh.vertices.size() * 0.25f);

            if (!useRetopology || targetVertexCount > maxRetopoAllowed) {
                ModelData tempModel;
                tempModel.meshes.push_back(rawMesh);
                MeshType oMesh = MeshProcessor::convertRawToOpenMesh(tempModel);

                MeshProcessor::decimateMesh(oMesh, targetVertexCount);
                MeshProcessor::extractRawFromOpenMesh(oMesh, sourceVerts, sourceInds);

                // If user didn't want quads, we are done
                if (!useRetopology) {
                    ok = true;
                    previewVerts = sourceVerts;
                    previewInds = sourceInds;
                }
            }

            // If the user requested Quads, run Instant Meshes on the current source
            if (useRetopology) {
                ok = RetopoProcessor::processRetopology(
                    sourceVerts, sourceInds,
                    previewVerts, previewInds, targetVertexCount);
            }
        }

        if (!ok && previewVerts.empty()) {
            previewVerts = rawMesh.vertices;
            previewInds = rawMesh.indices;
        }

        QMetaObject::invokeMethod(this, [this, v = std::move(previewVerts), i = std::move(previewInds)]() mutable {
            m_previewVertices = std::move(v);
            m_previewIndices = std::move(i);
            m_showingPreview = true;
            m_btnApply->setEnabled(true);
            m_btnAccept->setEnabled(true);
            RefreshViewportAndUI();
            setEnabled(true);
            });
        });
}

void MainWindow::OnApplyClicked()
{
    if (!m_showingPreview) return;

    // Overwrite the current base geometry with the previewed geometry
    m_currentModel.meshes[m_currentMeshIndex].vertices = m_previewVertices;
    m_currentModel.meshes[m_currentMeshIndex].indices = m_previewIndices;

    m_showingPreview = false;
    m_previewVertices.clear();
    m_previewIndices.clear();

    m_btnApply->setEnabled(false); // Disable until they preview again

    RefreshViewportAndUI();
}

void MainWindow::OnAcceptClicked()
{
    // Grab whichever geometry is currently active/visible
    const std::vector<Vertex>& verts = m_showingPreview ? m_previewVertices : m_currentModel.meshes[m_currentMeshIndex].vertices;
    const std::vector<unsigned int>& inds = m_showingPreview ? m_previewIndices : m_currentModel.meshes[m_currentMeshIndex].indices;

    unsigned int offset = static_cast<unsigned int>(m_retopoVertices.size());
    m_retopoVertices.insert(m_retopoVertices.end(), verts.begin(), verts.end());
    for (unsigned int idx : inds) m_retopoIndices.push_back(idx + offset);

    m_showingPreview = false;
    m_previewVertices.clear();
    m_previewIndices.clear();

    m_currentMeshIndex++; // Move to the next part
    m_btnApply->setEnabled(false);

    // Only disable accept if we're out of parts
    if (m_currentMeshIndex >= m_currentModel.meshes.size()) {
        m_btnAccept->setEnabled(false);
    }

    RefreshViewportAndUI();
}