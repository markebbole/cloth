#include "controller.h"
#include "mainwindow.h"
#include "simulation.h"
#include <QDebug>
#include <Eigen/Core>

using namespace Eigen;

Controller::Controller(int fps) : QThread(), mw_(NULL), fps_(fps), simtimer_(NULL)
{    
}

Controller::~Controller()
{
    if(simtimer_)
        delete simtimer_;
    delete sim_;
}

void Controller::initialize(MainWindow *mw)
{
    mw_ = mw;
    sim_ = new Simulation(params_);
    reset();
}

void Controller::initializeGL()
{
    sim_->initializeGL();
}

void Controller::run()
{
    simtimer_ = new QTimer();
    connect(simtimer_, SIGNAL(timeout()), this, SLOT(simTick()));
    simtimer_->start(1000/fps_);    
    exec();
}

void Controller::reset()
{
    params_ = SimParameters();
    QMetaObject::invokeMethod(mw_, "setUIFromParameters", Q_ARG(SimParameters, params_));
    clearScene();
}

void Controller::clearScene()
{
    QMetaObject::invokeMethod(mw_, "setUIFromParameters", Q_ARG(SimParameters, params_));
    sim_->clearScene();
}

void Controller::updateParameters(SimParameters params)
{
    params_ = params;
}

void Controller::renderObjects()
{
    sim_->renderObjects();
}

void Controller::simTick()
{
    simtimer_->blockSignals(true);
    if(params_.simRunning)
    {
        sim_->takeSimulationStep();
    }
    simtimer_->blockSignals(false);
}

void Controller::getCameraInfo(int body, Eigen::Vector3d &center, double &scale)
{
    center = sim_->getBodyPosition(body);
    scale = 1.0 / sim_->getBodyBoundingRadius(body);
}
