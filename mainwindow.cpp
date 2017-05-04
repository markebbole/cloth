#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "simparameters.h"
#include "controller.h"
#include <iostream>

using namespace std;

MainWindow::MainWindow(Controller &cont, int fps, QWidget *parent) :
    QMainWindow(parent),
    cont_(cont),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->GLWidget->setController(&cont);
    simRunning_ = false;
    renderTimer_ = new QTimer();
    connect(renderTimer_, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer_->start(1000/fps);
}

MainWindow::~MainWindow()
{
    delete renderTimer_;
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    close();
}

void MainWindow::setParametersFromUI()
{
    SimParameters params;

    params.simRunning = simRunning_;

    params.timeStep = ui->timeStepEdit->text().toDouble();
    params.clothWidth = ui->clothWidthEdit->text().toInt();

    //params.NewtonTolerance = ui->newtonTolEdit->text().toDouble();
    //params.NewtonMaxIters = ui->newtonMaxItersEdit->text().toInt();

    params.activeForces = 0;
    if(ui->gravityCheckBox->isChecked())
        params.activeForces |= SimParameters::F_GRAVITY;

    if(ui->collisionRepulsionCheckBox->isChecked())
        params.activeForces |= SimParameters::F_COLLISION_REPULSION;

    if(ui->stretchingCheckBox->isChecked())
        params.activeForces |= SimParameters::F_STRETCH;
    if(ui->shearingCheckBox->isChecked())
        params.activeForces |= SimParameters::F_SHEAR;
    if(ui->bendingCheckBox->isChecked())
        params.activeForces |= SimParameters::F_BEND;

    params.springStiffness  = ui->springEdit->text().toDouble();
    params.shearStiffness = ui->shearEdit->text().toDouble();
    params.bendStiffness = ui->bendEdit->text().toDouble();
    params.dampingStiffness = ui->dampEdit->text().toDouble();
    params.clothSideLen = ui->clothSideLenEdit->text().toDouble();
    params.gravityC = ui->gravityEdit->text().toDouble();
    params.clothDensity = ui->clothDensityEdit->text().toDouble();


    setUIFromParameters(params);
    QMetaObject::invokeMethod(&cont_, "updateParameters", Q_ARG(SimParameters, params));
}

void MainWindow::setUIFromParameters(const SimParameters &params)
{
    if(params.simRunning)
    {
        ui->startSimulationButton->setText(QString("Pause Simulation"));
        simRunning_ = true;
    }
    else
    {
        ui->startSimulationButton->setText(QString("Start Simulation"));
        simRunning_ = false;
    }

    ui->timeStepEdit->setText(QString::number(params.timeStep));
    ui->clothWidthEdit->setText(QString::number(params.clothWidth));
    ui->gravityCheckBox->setChecked(params.activeForces & SimParameters::F_GRAVITY);

    ui->stretchingCheckBox->setChecked(params.activeForces & SimParameters::F_STRETCH);

    ui->shearingCheckBox->setChecked(params.activeForces & SimParameters::F_SHEAR);
    ui->bendingCheckBox->setChecked(params.activeForces & SimParameters::F_BEND);
    ui->collisionRepulsionCheckBox->setChecked(params.activeForces & SimParameters::F_COLLISION_REPULSION);

    ui->springEdit->setText(QString::number(params.springStiffness));
    ui->shearEdit->setText(QString::number(params.shearStiffness));
    ui->bendEdit->setText(QString::number(params.bendStiffness));
    ui->dampEdit->setText(QString::number(params.dampingStiffness));
    ui->clothSideLenEdit->setText(QString::number(params.clothSideLen));
    ui->gravityEdit->setText(QString::number(params.gravityC));
    ui->clothDensityEdit->setText(QString::number(params.clothDensity));
}

void MainWindow::updateGL()
{
    ui->GLWidget->tick();
    ui->GLWidget->update();
}

void MainWindow::on_actionReset_Everything_triggered()
{
    QMetaObject::invokeMethod(&cont_, "reset");
}

void MainWindow::on_actionReset_triggered()
{
    QMetaObject::invokeMethod(&cont_, "clearScene");
}

void MainWindow::on_startSimulationButton_clicked()
{
    simRunning_ = !simRunning_;
    setParametersFromUI();
}

void MainWindow::on_timeStepEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_gravityCheckBox_clicked()
{
    setParametersFromUI();
}


void MainWindow::on_collisionRepulsionCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_stretchingCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_shearingCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_bendingCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_shearEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_bendEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_dampEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_clothWidthEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_clothSideLenEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_gravityEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_clothDensityEdit_editingFinished()
{
    setParametersFromUI();
}
