// ToolsDev.cpp : Defines the entry point for the application.
//

#include "framework.h"
#include "ToolsDev.h"

#include <QApplication>
#include <QSurfaceFormat>
#include "MainWindow.h"

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(hInstance);

    // Set OpenGL version before QApplication
    QSurfaceFormat fmt;
    fmt.setVersion(4, 5);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSamples(4);
    fmt.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(fmt);

    int argc = 0;
    QApplication app(argc, nullptr);

    MainWindow window;
    window.show();

    return app.exec();
}
