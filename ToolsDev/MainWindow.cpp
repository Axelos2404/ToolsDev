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

    m_btnPreview = new QPushButton("Preview Retopology");
    m_btnAccept = new QPushButton("Accept & Next Part");
    m_btnAccept->setEnabled(false);

    connect(m_btnPreview, &QPushButton::clicked, this, [this]() {
        OnApplyDecimationClicked(m_targetVertSpinBox->value());
        });

    connect(m_btnAccept, &QPushButton::clicked, this, [this]() {
        OnAcceptClicked();
        });

    dockLayout->addLayout(targetLayout);
    dockLayout->addWidget(m_targetVertSlider);
    dockLayout->addWidget(m_btnPreview);
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

    for (size_t i = 0; i < inds.size(); i += 3) {
        unsigned int i0 = inds[i] + 1, i1 = inds[i + 1] + 1, i2 = inds[i + 2] + 1;
        file << "f " << i0 << "/" << i0 << "/" << i0 << " " << i1 << "/" << i1 << "/" << i1 << " " << i2 << "/" << i2 << "/" << i2 << "\n";
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

        // 1. Finished parts (Only show if context checkbox is true)
        if (showContext) {
            for (size_t i = 0; i < m_currentMeshIndex; ++i) {
                displayMeshes.push_back(m_currentModel.meshes[i]);
            }
        }

        // 2. The active target part (Always show this!)
        MeshData activePart;
        activePart.vertices = m_showingPreview ? m_previewVertices : currentPart.vertices;
        activePart.indices = m_showingPreview ? m_previewIndices : currentPart.indices;
        displayMeshes.push_back(activePart);

        // 3. The raw future parts (Only show if context checkbox is true)
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

        m_statusLabel->setText("All parts processed! Exporting to C:\\Temp\\Final_StepByStep.obj");
        ExportToOBJ("C:\\Temp\\Final_StepByStep.obj", m_retopoVertices, m_retopoIndices);

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

    m_workerThread = std::thread([this, rawMesh, targetVertexCount]() {

        std::vector<Vertex> quadVerts;
        std::vector<unsigned int> quadInds;
        bool retopoOk = false;

        if (rawMesh.vertices.size() > 50 && (rawMesh.indices.size() / 3) > 20) {
            retopoOk = RetopoProcessor::processRetopology(
                rawMesh.vertices, rawMesh.indices,
                quadVerts, quadInds, targetVertexCount);
        }

        if (!retopoOk) {
            quadVerts = rawMesh.vertices;
            quadInds = rawMesh.indices;
        }

        QMetaObject::invokeMethod(this, [this, v = std::move(quadVerts), i = std::move(quadInds)]() mutable {
            m_previewVertices = std::move(v);
            m_previewIndices = std::move(i);
            m_showingPreview = true;
            m_btnAccept->setEnabled(true);
            RefreshViewportAndUI();
            setEnabled(true);
            });
        });
}

void MainWindow::OnAcceptClicked()
{
    if (!m_showingPreview) return;

    unsigned int offset = static_cast<unsigned int>(m_retopoVertices.size());
    m_retopoVertices.insert(m_retopoVertices.end(), m_previewVertices.begin(), m_previewVertices.end());
    for (unsigned int idx : m_previewIndices) m_retopoIndices.push_back(idx + offset);

    m_showingPreview = false;
    m_previewVertices.clear();
    m_previewIndices.clear();
    m_currentMeshIndex++;
    m_btnAccept->setEnabled(false);
    RefreshViewportAndUI();
}