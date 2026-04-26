#pragma once

#include <QMainWindow>
#include <QLabel>
#include <thread>
#include <atomic>
#include "ViewportWidget.h"
#include "ModelLoader.h"
#include "MeshProcessor.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
	~MainWindow();
	void OnApplyDecimationClicked(int targetVertexCount);
private slots:
    void OnFileOpen();

private:
    ViewportWidget* m_viewport = nullptr;
    QLabel* m_statusLabel = nullptr;
    ModelLoader m_loader;
    ModelData m_currentModel;
	std::thread m_workerThread;
};