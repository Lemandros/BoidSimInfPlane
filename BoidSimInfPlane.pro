#-------------------------------------------------
#
# Project created by QtCreator 2016-03-04T14:54:57
#
#-------------------------------------------------

QT       += core gui


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = BoidSimInfPlane
TEMPLATE = app

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    boidsim2d.cpp \
    qcustomplot.cpp \
    plotmainwindow.cpp \
    fft.cpp

HEADERS  += mainwindow.h \
    boidsim2d.h \
    qcustomplot.h \
    plotmainwindow.h \
    fft.h \
    FFTw/include/fftw3.h


FORMS    += mainwindow.ui \
    plotmainwindow.ui \
    fft.ui

QMAKE_CXXFLAGS += -fopenmp
QMAKE_CFLAGS_RELEASE += -fopenmp
LIBS += -fopenmp
unix: LIBS += -lfftw3 -lm


RESOURCES = resources.qrc
