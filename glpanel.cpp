#include "glpanel.h"
#include "controller.h"
#include <QMouseEvent>
#include <iostream>

using namespace Eigen;

GLPanel::GLPanel(QWidget *parent) :
    QGLWidget(parent), c_(), dragging_(false), scale_(3.0)
{
    oldpos_.setZero();
    cameraView_ = 0;
    cont_ = NULL;
    c_.setDefault3D();
    lightPos_[0] = 1.;
    lightPos_[1] = 0.;
    lightPos_[2] = 0.;
    lightPos_[3] = 0.;
    rot = 0.;
}

void GLPanel::setController(Controller *cont)
{
    cont_ = cont;
}

void GLPanel::initializeGL()
{
    assert(cont_);
    glShadeModel(GL_FLAT);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    GLfloat lmodel_ambient[] = { 0.06, 0.06, 0.06, 1.0 };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    cont_->initializeGL();
}

void GLPanel::resizeGL(int w, int h)
{
    c_.setPerpective(60.0, 1.0);
    c_.setViewport(w, h);
}

void GLPanel::paintGL()
{
    assert(cont_);
    //rot += .01;
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (0.0, 0.0, 0.0);

    c_.applyViewport();
    c_.applyProjection();

    Vector3d eye(0,0,0);
    /*double scale = 1.0;
    cont_->getCameraInfo(cameraView_, eye, scale);

    eye *= scale;

    double dz = exp(scale_);

    eye[2] = dz;*/
    //c_.setEye(eye);
    //eye[2] -= 5.0;
    c_.setCenter(eye);
    eye[2] = 5.;
    Vector3d axis(0., rot, 0.);
    eye = VectorMath::rotationMatrix(axis) * eye;
    c_.setEye(eye);
    Vector3d up(0,1,0);
    c_.setUp(up);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    c_.applyLookAt();
    //glScaled(scale,scale,scale);

    GLfloat lightColor0[] = {1.f, 1.f, 1.f, 1.0f};
    GLfloat ambientColor0[] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat lightPosition[4] = { (float)lightPos_[0], (float)lightPos_[1], (float)lightPos_[2], (float)lightPos_[3] };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glLightf( GL_LIGHT0, GL_CONSTANT_ATTENUATION, 1.0f );
    glLightf( GL_LIGHT0, GL_LINEAR_ATTENUATION , 0.0f );
    glLightf( GL_LIGHT0, GL_QUADRATIC_ATTENUATION , 0.0f );
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);    
    cont_->renderObjects();
}

void GLPanel::scaleMousePos(int x, int y, double &scaledx, double &scaledy) const
{
    int w, h;
    c_.getViewport(w,h);
    scaledx = 2.0 * x / double(w-1) - 1.0;
    scaledy = 2.0 * (h - y -1) / double(h-1) - 1.0;
}

void GLPanel::mousePressEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    Vector2d pos;
    scaleMousePos(x,y,pos[0],pos[1]);

    dragging_ = true;
    oldpos_ = pos;
}

void GLPanel::mouseMoveEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();
    Vector2d pos;
    scaleMousePos(x,y,pos[0],pos[1]);
    if(dragging_)
    {
        double dy = pos[1] - oldpos_[1];
        scale_ += dy;
        oldpos_ = pos;
    }
}

void GLPanel::mouseReleaseEvent(QMouseEvent *)
{
    dragging_ = false;
}

void GLPanel::keyPressEvent(QKeyEvent *ke)
{
    int cam = ke->key()-'0';
    if(cam >= 0 && cam <= 9)
        cameraView_ = cam;
}

void GLPanel::tick()
{
}

void GLPanel::keyReleaseEvent(QKeyEvent *)
{

}
