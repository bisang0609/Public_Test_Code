#-------------------------------------------------
#
# Project created by QtCreator 2022-01-05T17:22:25
#
#-------------------------------------------------

QT       += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
# message("QT_VERSION  : $$QT_VERSION ")

TARGET = VSLS_QTgui
TEMPLATE = app

CONFIG(release, debug|release):DEFINES += QT_NO_DEBUG_OUTPUT


# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
    OtaNervusDetector.cpp \
    agent_curas.cpp \
    imageprocessor.cpp \
        mainwindow.cpp \
    osdwidget.cpp \
    serialservice.cpp \
    serialworker.cpp \
    udpservice.cpp


INCLUDEPATH += $$PWD/../Libs/opencv/install/include
INCLUDEPATH += $$PWD/../Libs/opencv/install/include/opencv2

LIBS += -L$$PWD/../Libs/opencv/install/x64/mingw/bin/ -lopencv_core4100
LIBS += -L$$PWD/../Libs/opencv/install/x64/mingw/bin/ -lopencv_imgproc4100
LIBS += -L$$PWD/../Libs/opencv/install/x64/mingw/bin/ -lopencv_highgui4100
LIBS += -L$$PWD/../Libs/opencv/install/x64/mingw/bin/ -lopencv_imgcodecs4100



HEADERS  += mainwindow.h \
    OtaNervusDetector.h \
    agent_curas.h \
    common.h \
    imageprocessor.h \
    osdwidget.h \
    serialservice.h \
    serialworker.h \
    udpservice.h \
    version.h

FORMS    += mainwindow.ui

RESOURCES += \
    resources/resources.qrc

CONFIG(debug, debug|release): CONFIG += console
CONFIG += x86_64 #64bit 빌드


