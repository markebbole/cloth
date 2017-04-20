/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "glpanel.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QAction *actionReset;
    QAction *actionReset_Everything;
    QWidget *centralWidget;
    GLPanel *GLWidget;
    QFrame *parameterFrame;
    QGroupBox *simOptionsBox;
    QGroupBox *activeForcesBox;
    QCheckBox *gravityCheckBox;
    QLabel *springLabel;
    QLineEdit *springEdit;
    QLabel *shearLabel;
    QLineEdit *shearEdit;
    QLabel *bendLabel;
    QLineEdit *bendEdit;
    QLabel *dampLabel;
    QLineEdit *dampEdit;
    QCheckBox *collisionRepulsionCheckBox;
    QCheckBox *stretchingCheckBox;
    QCheckBox *shearingCheckBox;
    QCheckBox *bendingCheckBox;
    QLabel *clothWidthLabel;
    QLineEdit *clothWidthEdit;
    QLabel *clothSideLenLabel;
    QLineEdit *clothSideLenEdit;
    QPushButton *startSimulationButton;
    QGroupBox *SimParametersBox;
    QLabel *timeStepLabel;
    QLineEdit *timeStepEdit;
    QLabel *moonPiecesButton;
    QLabel *explosionMag;
    QLineEdit *moonPieceEdit;
    QLineEdit *explosionMagEdit;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuScene;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1204, 800);
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionReset = new QAction(MainWindow);
        actionReset->setObjectName(QStringLiteral("actionReset"));
        actionReset_Everything = new QAction(MainWindow);
        actionReset_Everything->setObjectName(QStringLiteral("actionReset_Everything"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        GLWidget = new GLPanel(centralWidget);
        GLWidget->setObjectName(QStringLiteral("GLWidget"));
        GLWidget->setGeometry(QRect(10, 0, 731, 731));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(GLWidget->sizePolicy().hasHeightForWidth());
        GLWidget->setSizePolicy(sizePolicy);
        GLWidget->setFocusPolicy(Qt::StrongFocus);
        parameterFrame = new QFrame(centralWidget);
        parameterFrame->setObjectName(QStringLiteral("parameterFrame"));
        parameterFrame->setGeometry(QRect(740, -220, 441, 731));
        parameterFrame->setFrameShape(QFrame::StyledPanel);
        parameterFrame->setFrameShadow(QFrame::Raised);
        simOptionsBox = new QGroupBox(parameterFrame);
        simOptionsBox->setObjectName(QStringLiteral("simOptionsBox"));
        simOptionsBox->setGeometry(QRect(0, 250, 431, 471));
        simOptionsBox->setMaximumSize(QSize(16777215, 500));
        activeForcesBox = new QGroupBox(simOptionsBox);
        activeForcesBox->setObjectName(QStringLiteral("activeForcesBox"));
        activeForcesBox->setGeometry(QRect(0, 100, 431, 230));
        activeForcesBox->setMaximumSize(QSize(16777215, 230));
        gravityCheckBox = new QCheckBox(activeForcesBox);
        gravityCheckBox->setObjectName(QStringLiteral("gravityCheckBox"));
        gravityCheckBox->setGeometry(QRect(30, 30, 97, 21));
        springLabel = new QLabel(activeForcesBox);
        springLabel->setObjectName(QStringLiteral("springLabel"));
        springLabel->setGeometry(QRect(230, 50, 121, 21));
        springEdit = new QLineEdit(activeForcesBox);
        springEdit->setObjectName(QStringLiteral("springEdit"));
        springEdit->setGeometry(QRect(360, 50, 61, 21));
        shearLabel = new QLabel(activeForcesBox);
        shearLabel->setObjectName(QStringLiteral("shearLabel"));
        shearLabel->setGeometry(QRect(230, 70, 111, 21));
        shearEdit = new QLineEdit(activeForcesBox);
        shearEdit->setObjectName(QStringLiteral("shearEdit"));
        shearEdit->setGeometry(QRect(360, 70, 61, 21));
        bendLabel = new QLabel(activeForcesBox);
        bendLabel->setObjectName(QStringLiteral("bendLabel"));
        bendLabel->setGeometry(QRect(230, 90, 111, 21));
        bendEdit = new QLineEdit(activeForcesBox);
        bendEdit->setObjectName(QStringLiteral("bendEdit"));
        bendEdit->setGeometry(QRect(360, 90, 61, 21));
        dampLabel = new QLabel(activeForcesBox);
        dampLabel->setObjectName(QStringLiteral("dampLabel"));
        dampLabel->setGeometry(QRect(230, 120, 111, 21));
        dampEdit = new QLineEdit(activeForcesBox);
        dampEdit->setObjectName(QStringLiteral("dampEdit"));
        dampEdit->setGeometry(QRect(360, 120, 61, 21));
        collisionRepulsionCheckBox = new QCheckBox(activeForcesBox);
        collisionRepulsionCheckBox->setObjectName(QStringLiteral("collisionRepulsionCheckBox"));
        collisionRepulsionCheckBox->setGeometry(QRect(30, 50, 191, 21));
        stretchingCheckBox = new QCheckBox(activeForcesBox);
        stretchingCheckBox->setObjectName(QStringLiteral("stretchingCheckBox"));
        stretchingCheckBox->setGeometry(QRect(30, 70, 191, 21));
        shearingCheckBox = new QCheckBox(activeForcesBox);
        shearingCheckBox->setObjectName(QStringLiteral("shearingCheckBox"));
        shearingCheckBox->setGeometry(QRect(30, 90, 191, 21));
        bendingCheckBox = new QCheckBox(activeForcesBox);
        bendingCheckBox->setObjectName(QStringLiteral("bendingCheckBox"));
        bendingCheckBox->setGeometry(QRect(30, 110, 191, 21));
        clothWidthLabel = new QLabel(activeForcesBox);
        clothWidthLabel->setObjectName(QStringLiteral("clothWidthLabel"));
        clothWidthLabel->setGeometry(QRect(20, 190, 111, 21));
        clothWidthEdit = new QLineEdit(activeForcesBox);
        clothWidthEdit->setObjectName(QStringLiteral("clothWidthEdit"));
        clothWidthEdit->setGeometry(QRect(120, 190, 81, 21));
        clothSideLenLabel = new QLabel(activeForcesBox);
        clothSideLenLabel->setObjectName(QStringLiteral("clothSideLenLabel"));
        clothSideLenLabel->setGeometry(QRect(20, 210, 111, 21));
        clothSideLenEdit = new QLineEdit(activeForcesBox);
        clothSideLenEdit->setObjectName(QStringLiteral("clothSideLenEdit"));
        clothSideLenEdit->setGeometry(QRect(120, 210, 81, 21));
        startSimulationButton = new QPushButton(simOptionsBox);
        startSimulationButton->setObjectName(QStringLiteral("startSimulationButton"));
        startSimulationButton->setGeometry(QRect(0, 50, 181, 27));
        SimParametersBox = new QGroupBox(simOptionsBox);
        SimParametersBox->setObjectName(QStringLiteral("SimParametersBox"));
        SimParametersBox->setGeometry(QRect(210, 30, 206, 89));
        timeStepLabel = new QLabel(SimParametersBox);
        timeStepLabel->setObjectName(QStringLiteral("timeStepLabel"));
        timeStepLabel->setGeometry(QRect(10, 30, 81, 21));
        timeStepEdit = new QLineEdit(SimParametersBox);
        timeStepEdit->setObjectName(QStringLiteral("timeStepEdit"));
        timeStepEdit->setGeometry(QRect(140, 30, 61, 21));
        moonPiecesButton = new QLabel(SimParametersBox);
        moonPiecesButton->setObjectName(QStringLiteral("moonPiecesButton"));
        moonPiecesButton->setGeometry(QRect(10, 120, 121, 21));
        explosionMag = new QLabel(SimParametersBox);
        explosionMag->setObjectName(QStringLiteral("explosionMag"));
        explosionMag->setGeometry(QRect(10, 140, 131, 21));
        moonPieceEdit = new QLineEdit(SimParametersBox);
        moonPieceEdit->setObjectName(QStringLiteral("moonPieceEdit"));
        moonPieceEdit->setGeometry(QRect(140, 120, 61, 21));
        explosionMagEdit = new QLineEdit(SimParametersBox);
        explosionMagEdit->setObjectName(QStringLiteral("explosionMagEdit"));
        explosionMagEdit->setGeometry(QRect(140, 140, 61, 21));
        activeForcesBox->raise();
        startSimulationButton->raise();
        SimParametersBox->raise();
        clothSideLenEdit->raise();
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1204, 25));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuScene = new QMenu(menuBar);
        menuScene->setObjectName(QStringLiteral("menuScene"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuScene->menuAction());
        menuFile->addAction(actionExit);
        menuScene->addAction(actionReset);
        menuScene->addAction(actionReset_Everything);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Seveneves", 0));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0));
        actionReset->setText(QApplication::translate("MainWindow", "Clear Scene", 0));
        actionReset_Everything->setText(QApplication::translate("MainWindow", "Reset Everything", 0));
        simOptionsBox->setTitle(QApplication::translate("MainWindow", "Simulation Options", 0));
        activeForcesBox->setTitle(QApplication::translate("MainWindow", "Active Forces", 0));
        gravityCheckBox->setText(QApplication::translate("MainWindow", "Gravity", 0));
        springLabel->setText(QApplication::translate("MainWindow", "Spring Stiffness", 0));
        shearLabel->setText(QApplication::translate("MainWindow", "Shear Stiffness:", 0));
        bendLabel->setText(QApplication::translate("MainWindow", "Bend Stiffness:", 0));
        dampLabel->setText(QApplication::translate("MainWindow", "Damp Stiffness:", 0));
        collisionRepulsionCheckBox->setText(QApplication::translate("MainWindow", "Collision Repulsion", 0));
        stretchingCheckBox->setText(QApplication::translate("MainWindow", "Stretching", 0));
        shearingCheckBox->setText(QApplication::translate("MainWindow", "Shearing", 0));
        bendingCheckBox->setText(QApplication::translate("MainWindow", "Bending", 0));
        clothWidthLabel->setText(QApplication::translate("MainWindow", "Cloth Width:", 0));
        clothSideLenLabel->setText(QApplication::translate("MainWindow", "Cloth Side Len:", 0));
        startSimulationButton->setText(QApplication::translate("MainWindow", "Start Simulation", 0));
        SimParametersBox->setTitle(QApplication::translate("MainWindow", "Parameters", 0));
        timeStepLabel->setText(QApplication::translate("MainWindow", "Time Step:", 0));
        moonPiecesButton->setText(QApplication::translate("MainWindow", "Moon Pieces:", 0));
        explosionMag->setText(QApplication::translate("MainWindow", "Explosion Mag:", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
        menuScene->setTitle(QApplication::translate("MainWindow", "Scene", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
