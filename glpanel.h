#ifndef GLPANEL_H
#define GLPANEL_H

#include <QGLWidget>
#include "camera.h"
#include "vectormath.h"

class Controller;

class GLPanel : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLPanel(QWidget *parent = 0);
    void setController(Controller *cont);

    void tick();

signals:

public slots:
    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent *me);
    virtual void mouseMoveEvent(QMouseEvent *me);
    virtual void mouseReleaseEvent(QMouseEvent *me);

    virtual void keyPressEvent(QKeyEvent *ke);
    virtual void keyReleaseEvent(QKeyEvent *ke);

private:
    void scaleMousePos(int x, int y, double &scaledx, double &scaledy) const;

    Controller *cont_;

    Camera c_;
    Eigen::Vector4d lightPos_;

    bool dragging_;
    Eigen::Vector2d oldpos_;
    double scale_;
    double rotLR;

    int cameraView_;
};

#endif // GLPANEL_H
