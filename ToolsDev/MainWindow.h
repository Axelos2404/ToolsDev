#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QCheckBox>
#include <thread>
#include <atomic>
#include "ViewportWidget.h"
#include "ModelLoader.h"
#include "MeshProcessor.h"
#include "RetopoProcessor.h"
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
    void OnApplyDecimationClicked(int targetVertexCount);
    void OnApplyClicked();
    void OnAcceptClicked();

private slots:
    void OnFileOpen();

private:
    ViewportWidget* m_viewport = nullptr;
    QLabel* m_statusLabel = nullptr;

    QPushButton* m_btnPreview = nullptr;
    QPushButton* m_btnApply = nullptr;
    QPushButton* m_btnAccept = nullptr;

    QSlider* m_targetVertSlider = nullptr;
    QSpinBox* m_targetVertSpinBox = nullptr;
    QCheckBox* m_chkShowContext = nullptr;
    QCheckBox* m_chkUseInstantMeshes = nullptr;

    ModelLoader m_loader;
    ModelData m_currentModel;
    std::thread m_workerThread;

    int m_currentMeshIndex = 0;
    std::vector<Vertex> m_retopoVertices;
    std::vector<unsigned int> m_retopoIndices;

    bool m_showingPreview = false;
    std::vector<Vertex> m_previewVertices;
    std::vector<unsigned int> m_previewIndices;

    void RefreshViewportAndUI();
};