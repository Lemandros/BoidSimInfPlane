#-------------------------------------------------
#
# Project created by QtCreator 2016-03-04T14:54:57
#
#-------------------------------------------------

QT       += core gui
QMAKE_CFLAGS_RELEASE += -fopenmp
LIBS += -fopenmp

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = BoidSimInfPlane
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    boidsim2d.cpp \
    qcustomplot.cpp \
    plotmainwindow.cpp

HEADERS  += mainwindow.h \
    boidsim2d.h \
    qcustomplot.h \
    plotmainwindow.h

FORMS    += mainwindow.ui \
    plotmainwindow.ui

QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

RESOURCES = resources.qrc
