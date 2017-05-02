#-------------------------------------------------
#
# Project created by QtCreator 2014-09-03T16:42:53
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = cloth
TEMPLATE = app

INCLUDEPATH += $$_PRO_FILE_PWD_/eigen/
win32 {
    LIBS += -lopengl32
}

unix {
    LIBS += -lGLU -Lexact-ccd/ -lexact-ccd
    QMAKE_CXXFLAGS += -std=c++11 -g
}

SOURCES += main.cpp\
        mainwindow.cpp \
    glpanel.cpp \
    controller.cpp \
    simulation.cpp \
    simparameters.cpp \
    vectormath.cpp \
    camera.cpp \
    collisiondetection.cpp \
    clothtemplate.cpp \
    clothinstance.cpp \
    obstacle.cpp

HEADERS  += mainwindow.h \
    glpanel.h \
    controller.h \
    simulation.h \
    simparameters.h \
    vectormath.h \
    camera.h \
    distance.h \
    collisiondetection.h \
    clothtemplate.h \
    clothinstance.h \ 
    obstacle.h \
    exact-ccd/rootparitycollisiontest.h

FORMS    += mainwindow.ui

CONFIG += c++11
