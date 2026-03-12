#include "ViewportWidget.h"
#include <cmath>
#include <algorithm>
#include <limits>

static const char* vertexShaderSrc = R"(
#version 450 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aTexCoord;

uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat3 uNormalMat;

out vec3 vNormal;
out vec3 vFragPos;

void main()
{
    gl_Position = uMVP * vec4(aPos, 1.0);
    vFragPos = vec3(uModel * vec4(aPos, 1.0));
    vNormal = normalize(uNormalMat * aNormal);
}
)";

static const char* fragmentShaderSrc = R"(
#version 450 core
in vec3 vNormal;
in vec3 vFragPos;
out vec4 FragColor;

void main()
{
    vec3 normal = normalize(vNormal);
    vec3 lightDir1 = normalize(vec3(1.0, 1.0, 1.0));
    vec3 lightDir2 = normalize(vec3(-1.0, 0.8, -0.5));
    vec3 baseColor = vec3(0.7, 0.7, 0.75);

    float diff1 = max(dot(normal, lightDir1), 0.0);
    float diff2 = max(dot(normal, lightDir2), 0.0);
    float ambient = 0.5;
    vec3 color = baseColor * (ambient + diff1 * 0.5 + diff2 * 0.4);

    FragColor = vec4(color, 1.0);
}
)";

ViewportWidget::ViewportWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(640, 480);
    setFocusPolicy(Qt::StrongFocus);
}

ViewportWidget::~ViewportWidget()
{
    makeCurrent();
    CleanupGpuMeshes();
    doneCurrent();
}

void ViewportWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.18f, 0.18f, 0.2f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    m_shader.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSrc);
    m_shader.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSrc);
    m_shader.link();
}

void ViewportWidget::resizeGL(int w, int h)
{
    m_aspectRatio = float(w) / float(h ? h : 1);
}

void ViewportWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Dynamic near/far planes based on zoom distance
    float nearPlane = m_zoom * 0.001f;
    float farPlane  = m_zoom * 100.0f;
    nearPlane = std::max(nearPlane, 0.001f);

    m_projection.setToIdentity();
    m_projection.perspective(45.0f, m_aspectRatio, nearPlane, farPlane);

    m_view.setToIdentity();
    m_view.translate(0.0f, 0.0f, -m_zoom);
    m_view.rotate(m_rotX, 1.0f, 0.0f, 0.0f);
    m_view.rotate(m_rotY, 0.0f, 1.0f, 0.0f);
    m_view.translate(-m_target);

    QMatrix4x4 model;
    QMatrix4x4 mvp = m_projection * m_view * model;
    QMatrix3x3 normalMat = model.normalMatrix();

    m_shader.bind();
    m_shader.setUniformValue("uMVP", mvp);
    m_shader.setUniformValue("uModel", model);
    m_shader.setUniformValue("uNormalMat", normalMat);

    for (const auto& gpu : m_gpuMeshes)
    {
        glBindVertexArray(gpu.vao);
        glDrawElements(GL_TRIANGLES, gpu.indexCount, GL_UNSIGNED_INT, nullptr);
    }

    glBindVertexArray(0);
    m_shader.release();
}

void ViewportWidget::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePos = event->position();
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event)
{
    QPointF delta = event->position() - m_lastMousePos;
    m_lastMousePos = event->position();

    if (event->buttons() & Qt::MiddleButton)
    {
        m_rotY += float(delta.x()) * 0.5f;
        m_rotX += float(delta.y()) * 0.5f;
        m_rotX = qBound(-89.0f, m_rotX, 89.0f);
        update();
    }
}

void ViewportWidget::wheelEvent(QWheelEvent* event)
{
    m_zoom -= event->angleDelta().y() * 0.005f * m_zoom;
    m_zoom = qBound(0.01f, m_zoom, 100000.0f);
    update();
}

void ViewportWidget::SetModel(const ModelData& model)
{
    makeCurrent();
    CleanupGpuMeshes();

    for (const auto& mesh : model.meshes)
    {
        UploadMesh(mesh);
    }

    FitCameraToModel(model);

    doneCurrent();
    update();
}

void ViewportWidget::FitCameraToModel(const ModelData& model)
{
    if (model.meshes.empty())
        return;

    // Compute bounding box
    constexpr float maxF = std::numeric_limits<float>::max();
    QVector3D bboxMin(maxF, maxF, maxF);
    QVector3D bboxMax(-maxF, -maxF, -maxF);

    for (const auto& mesh : model.meshes)
    {
        for (const auto& v : mesh.vertices)
        {
            bboxMin.setX(std::min(bboxMin.x(), v.position[0]));
            bboxMin.setY(std::min(bboxMin.y(), v.position[1]));
            bboxMin.setZ(std::min(bboxMin.z(), v.position[2]));
            bboxMax.setX(std::max(bboxMax.x(), v.position[0]));
            bboxMax.setY(std::max(bboxMax.y(), v.position[1]));
            bboxMax.setZ(std::max(bboxMax.z(), v.position[2]));
        }
    }

    // Center the orbit target on the model
    m_target = (bboxMin + bboxMax) * 0.5f;

    // Set zoom to fit the bounding sphere
    float radius = (bboxMax - bboxMin).length() * 0.5f;
    float fovRad = qDegreesToRadians(45.0f * 0.5f);
    m_zoom = radius / std::tan(fovRad) * 1.5f;  // 1.5x padding

    m_rotX = 30.0f;
    m_rotY = -45.0f;
}

void ViewportWidget::UploadMesh(const MeshData& mesh)
{
    GpuMesh gpu;

    glCreateVertexArrays(1, &gpu.vao);
    glCreateBuffers(1, &gpu.vbo);
    glCreateBuffers(1, &gpu.ebo);

    glNamedBufferStorage(gpu.vbo,
        mesh.vertices.size() * sizeof(Vertex),
        mesh.vertices.data(), 0);

    glNamedBufferStorage(gpu.ebo,
        mesh.indices.size() * sizeof(unsigned int),
        mesh.indices.data(), 0);

    glVertexArrayVertexBuffer(gpu.vao, 0, gpu.vbo, 0, sizeof(Vertex));
    glVertexArrayElementBuffer(gpu.vao, gpu.ebo);

    // position
    glEnableVertexArrayAttrib(gpu.vao, 0);
    glVertexArrayAttribFormat(gpu.vao, 0, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, position));
    glVertexArrayAttribBinding(gpu.vao, 0, 0);

    // normal
    glEnableVertexArrayAttrib(gpu.vao, 1);
    glVertexArrayAttribFormat(gpu.vao, 1, 3, GL_FLOAT, GL_FALSE, offsetof(Vertex, normal));
    glVertexArrayAttribBinding(gpu.vao, 1, 0);

    // texcoord
    glEnableVertexArrayAttrib(gpu.vao, 2);
    glVertexArrayAttribFormat(gpu.vao, 2, 2, GL_FLOAT, GL_FALSE, offsetof(Vertex, texCoord));
    glVertexArrayAttribBinding(gpu.vao, 2, 0);

    gpu.indexCount = static_cast<GLsizei>(mesh.indices.size());
    m_gpuMeshes.push_back(gpu);
}

void ViewportWidget::CleanupGpuMeshes()
{
    for (auto& gpu : m_gpuMeshes)
    {
        glDeleteVertexArrays(1, &gpu.vao);
        glDeleteBuffers(1, &gpu.vbo);
        glDeleteBuffers(1, &gpu.ebo);
    }
    m_gpuMeshes.clear();
}