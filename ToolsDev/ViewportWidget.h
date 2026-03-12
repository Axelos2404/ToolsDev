#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPointF>
#include <QVector3D>
#include <vector>
#include "Mesh.h"

struct GpuMesh
{
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    GLsizei indexCount = 0;
};

class ViewportWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

    void SetModel(const ModelData& model);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void CleanupGpuMeshes();
    void UploadMesh(const MeshData& mesh);
    void FitCameraToModel(const ModelData& model);

    QOpenGLShaderProgram m_shader;
    std::vector<GpuMesh> m_gpuMeshes;
    QMatrix4x4 m_projection;
    QMatrix4x4 m_view;

    // Orbit camera
    float m_rotX = 30.0f;
    float m_rotY = -45.0f;
    float m_zoom = 5.0f;
    float m_aspectRatio = 1.0f;
    QVector3D m_target;
    QPointF m_lastMousePos;
};