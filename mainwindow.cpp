#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "simparameters.h"
#include "controller.h"
#include <iostream>

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
    params.NewtonTolerance = ui->newtonTolEdit->text().toDouble();
    params.NewtonMaxIters = ui->newtonMaxItersEdit->text().toInt();

    params.activeForces = 0;
    if(ui->gravityCheckBox->isChecked())
        params.activeForces |= SimParameters::F_GRAVITY;


    if(ui->collisionPenaltyCheckBox->isChecked())
        params.activeForces |= SimParameters::F_PENALTY;

    params.penaltyStiffness = ui->penaltyStiffnessEdit->text().toDouble();

    if(ui->collisionImpulsesCheckBox->isChecked())
        params.activeForces |= SimParameters::F_IMPULSE;

    params.CoR = ui->CoREdit->text().toDouble();

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
    ui->newtonTolEdit->setText(QString::number(params.NewtonTolerance));
    ui->newtonMaxItersEdit->setText(QString::number(params.NewtonMaxIters));

    ui->gravityCheckBox->setChecked(params.activeForces & SimParameters::F_GRAVITY);

    ui->collisionPenaltyCheckBox->setChecked(params.activeForces & SimParameters::F_PENALTY);
    ui->penaltyStiffnessEdit->setText(QString::number(params.penaltyStiffness));

    ui->collisionImpulsesCheckBox->setChecked(params.activeForces & SimParameters::F_IMPULSE);
    ui->CoREdit->setText(QString::number(params.CoR));
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

void MainWindow::on_newtonTolEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonMaxItersEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_gravityCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_collisionPenaltyCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_penaltyStiffnessEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_collisionImpulsesCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_CoREdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_velTreshEdit_editingFinished()
{
    setParametersFromUI();
}
