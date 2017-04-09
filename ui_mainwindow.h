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
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
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
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *simOptionsBox;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *horizontalLayout;
    QGroupBox *SimulationBox;
    QPushButton *startSimulationButton;
    QGroupBox *SimParametersBox;
    QLabel *timeStepLabel;
    QLabel *newtonTolLabel;
    QLabel *newtonMaxItersLabel;
    QLineEdit *timeStepEdit;
    QLineEdit *newtonTolEdit;
    QLineEdit *newtonMaxItersEdit;
    QLabel *moonPiecesButton;
    QLabel *explosionMag;
    QLineEdit *moonPieceEdit;
    QLineEdit *explosionMagEdit;
    QGroupBox *activeForcesBox;
    QCheckBox *gravityCheckBox;
    QCheckBox *collisionPenaltyCheckBox;
    QLabel *penaltyStiffnessLabel;
    QLineEdit *penaltyStiffnessEdit;
    QCheckBox *collisionImpulsesCheckBox;
    QLabel *CoRLabel;
    QLineEdit *CoREdit;
    QLabel *velTreshLabel;
    QLineEdit *velTreshEdit;
    QCheckBox *gravityCheckBox_2;
    QGroupBox *UIOptionsBox;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuScene;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1200, 800);
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
        parameterFrame->setGeometry(QRect(749, -1, 441, 731));
        parameterFrame->setFrameShape(QFrame::StyledPanel);
        parameterFrame->setFrameShadow(QFrame::Raised);
        verticalLayoutWidget = new QWidget(parameterFrame);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(9, -1, 431, 731));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        simOptionsBox = new QGroupBox(verticalLayoutWidget);
        simOptionsBox->setObjectName(QStringLiteral("simOptionsBox"));
        simOptionsBox->setMaximumSize(QSize(16777215, 220));
        horizontalLayoutWidget = new QWidget(simOptionsBox);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(9, 19, 421, 91));
        horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        SimulationBox = new QGroupBox(horizontalLayoutWidget);
        SimulationBox->setObjectName(QStringLiteral("SimulationBox"));
        startSimulationButton = new QPushButton(SimulationBox);
        startSimulationButton->setObjectName(QStringLiteral("startSimulationButton"));
        startSimulationButton->setGeometry(QRect(10, 40, 181, 27));

        horizontalLayout->addWidget(SimulationBox);

        SimParametersBox = new QGroupBox(horizontalLayoutWidget);
        SimParametersBox->setObjectName(QStringLiteral("SimParametersBox"));
        timeStepLabel = new QLabel(SimParametersBox);
        timeStepLabel->setObjectName(QStringLiteral("timeStepLabel"));
        timeStepLabel->setGeometry(QRect(10, 30, 81, 21));
        newtonTolLabel = new QLabel(SimParametersBox);
        newtonTolLabel->setObjectName(QStringLiteral("newtonTolLabel"));
        newtonTolLabel->setGeometry(QRect(10, 50, 131, 21));
        newtonMaxItersLabel = new QLabel(SimParametersBox);
        newtonMaxItersLabel->setObjectName(QStringLiteral("newtonMaxItersLabel"));
        newtonMaxItersLabel->setGeometry(QRect(10, 70, 131, 21));
        timeStepEdit = new QLineEdit(SimParametersBox);
        timeStepEdit->setObjectName(QStringLiteral("timeStepEdit"));
        timeStepEdit->setGeometry(QRect(140, 30, 61, 21));
        newtonTolEdit = new QLineEdit(SimParametersBox);
        newtonTolEdit->setObjectName(QStringLiteral("newtonTolEdit"));
        newtonTolEdit->setGeometry(QRect(140, 50, 61, 21));
        newtonMaxItersEdit = new QLineEdit(SimParametersBox);
        newtonMaxItersEdit->setObjectName(QStringLiteral("newtonMaxItersEdit"));
        newtonMaxItersEdit->setGeometry(QRect(140, 70, 61, 21));
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

        horizontalLayout->addWidget(SimParametersBox);


        verticalLayout->addWidget(simOptionsBox);

        activeForcesBox = new QGroupBox(verticalLayoutWidget);
        activeForcesBox->setObjectName(QStringLiteral("activeForcesBox"));
        activeForcesBox->setMaximumSize(QSize(16777215, 170));
        gravityCheckBox = new QCheckBox(activeForcesBox);
        gravityCheckBox->setObjectName(QStringLiteral("gravityCheckBox"));
        gravityCheckBox->setGeometry(QRect(30, 30, 97, 21));
        collisionPenaltyCheckBox = new QCheckBox(activeForcesBox);
        collisionPenaltyCheckBox->setObjectName(QStringLiteral("collisionPenaltyCheckBox"));
        collisionPenaltyCheckBox->setGeometry(QRect(30, 50, 131, 21));
        penaltyStiffnessLabel = new QLabel(activeForcesBox);
        penaltyStiffnessLabel->setObjectName(QStringLiteral("penaltyStiffnessLabel"));
        penaltyStiffnessLabel->setGeometry(QRect(230, 50, 61, 21));
        penaltyStiffnessEdit = new QLineEdit(activeForcesBox);
        penaltyStiffnessEdit->setObjectName(QStringLiteral("penaltyStiffnessEdit"));
        penaltyStiffnessEdit->setGeometry(QRect(360, 50, 61, 21));
        collisionImpulsesCheckBox = new QCheckBox(activeForcesBox);
        collisionImpulsesCheckBox->setObjectName(QStringLiteral("collisionImpulsesCheckBox"));
        collisionImpulsesCheckBox->setGeometry(QRect(30, 70, 131, 21));
        CoRLabel = new QLabel(activeForcesBox);
        CoRLabel->setObjectName(QStringLiteral("CoRLabel"));
        CoRLabel->setGeometry(QRect(230, 70, 61, 21));
        CoREdit = new QLineEdit(activeForcesBox);
        CoREdit->setObjectName(QStringLiteral("CoREdit"));
        CoREdit->setGeometry(QRect(360, 70, 61, 21));
        velTreshLabel = new QLabel(activeForcesBox);
        velTreshLabel->setObjectName(QStringLiteral("velTreshLabel"));
        velTreshLabel->setGeometry(QRect(230, 90, 81, 21));
        velTreshEdit = new QLineEdit(activeForcesBox);
        velTreshEdit->setObjectName(QStringLiteral("velTreshEdit"));
        velTreshEdit->setGeometry(QRect(360, 90, 61, 21));
        gravityCheckBox_2 = new QCheckBox(activeForcesBox);
        gravityCheckBox_2->setObjectName(QStringLiteral("gravityCheckBox_2"));
        gravityCheckBox_2->setGeometry(QRect(30, 30, 97, 21));
        gravityCheckBox->raise();
        collisionPenaltyCheckBox->raise();
        penaltyStiffnessLabel->raise();
        penaltyStiffnessEdit->raise();
        collisionImpulsesCheckBox->raise();
        CoRLabel->raise();
        CoREdit->raise();
        velTreshLabel->raise();
        velTreshEdit->raise();
        gravityCheckBox_2->raise();

        verticalLayout->addWidget(activeForcesBox);

        UIOptionsBox = new QGroupBox(verticalLayoutWidget);
        UIOptionsBox->setObjectName(QStringLiteral("UIOptionsBox"));

        verticalLayout->addWidget(UIOptionsBox);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1200, 25));
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
        SimulationBox->setTitle(QApplication::translate("MainWindow", "Simulation Controls", 0));
        startSimulationButton->setText(QApplication::translate("MainWindow", "Start Simulation", 0));
        SimParametersBox->setTitle(QApplication::translate("MainWindow", "Parameters", 0));
        timeStepLabel->setText(QApplication::translate("MainWindow", "Time Step:", 0));
        newtonTolLabel->setText(QApplication::translate("MainWindow", "Newton Tolerance:", 0));
        newtonMaxItersLabel->setText(QApplication::translate("MainWindow", "Newton Max Iters:", 0));
        moonPiecesButton->setText(QApplication::translate("MainWindow", "Moon Pieces:", 0));
        explosionMag->setText(QApplication::translate("MainWindow", "Explosion Mag:", 0));
        activeForcesBox->setTitle(QApplication::translate("MainWindow", "Active Forces", 0));
        gravityCheckBox->setText(QApplication::translate("MainWindow", "Gravity", 0));
        collisionPenaltyCheckBox->setText(QApplication::translate("MainWindow", "Collision Penalty Force", 0));
        penaltyStiffnessLabel->setText(QApplication::translate("MainWindow", "Stiffness:", 0));
        collisionImpulsesCheckBox->setText(QApplication::translate("MainWindow", "Collision Impulses", 0));
        CoRLabel->setText(QApplication::translate("MainWindow", "CoR:", 0));
        velTreshLabel->setText(QApplication::translate("MainWindow", "Vel Threshold:", 0));
        gravityCheckBox_2->setText(QApplication::translate("MainWindow", "Gravity", 0));
        UIOptionsBox->setTitle(QApplication::translate("MainWindow", "UI Options", 0));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0));
        menuScene->setTitle(QApplication::translate("MainWindow", "Scene", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
