#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

class Controller;
struct SimParameters;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(Controller &cont, int fps, QWidget *parent = 0);
    ~MainWindow();       

public slots:
    void setUIFromParameters(const SimParameters &params);

private slots:
    void updateGL();

    void on_actionExit_triggered();

    void on_actionReset_Everything_triggered();

    void on_actionReset_triggered();

    void on_startSimulationButton_clicked();

    void on_timeStepEdit_editingFinished();

    void on_gravityCheckBox_clicked();

    void on_collisionRepulsionCheckBox_clicked();

    void on_stretchingCheckBox_clicked();

    void on_shearingCheckBox_clicked();

    void on_bendingCheckBox_clicked();

    void on_shearEdit_editingFinished();

    void on_bendEdit_editingFinished();

    void on_dampEdit_editingFinished();

    void on_clothWidthEdit_editingFinished();

    void on_clothSideLenEdit_editingFinished();

private:
    Controller &cont_;
    Ui::MainWindow *ui;
    bool simRunning_;
    QTimer *renderTimer_;

    void setParametersFromUI();
};

#endif // MAINWINDOW_H
