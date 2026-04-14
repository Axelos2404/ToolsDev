#include "MainWindow.h"
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
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

    // Status bar
    m_statusLabel = new QLabel("Ready");
    statusBar()->addWidget(m_statusLabel);
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