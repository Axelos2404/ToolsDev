#pragma once

#include <QMainWindow>
#include <QLabel>
#include "ViewportWidget.h"
#include "ModelLoader.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void OnFileOpen();

private:
    ViewportWidget* m_viewport = nullptr;
    QLabel* m_statusLabel = nullptr;
    ModelLoader m_loader;
    ModelData m_currentModel;
};