TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

INCLUDEPATH+=/opt/ros/indigo/include
INCLUDEPATH+=/opt/ros/indigo/include/opencv-3.2.0-dev
INCLUDEPATH+=../include

SOURCES += \
    ../src/cvtrafficlightconfig.cpp \
    ../src/cvtrafficlighttune.cpp

DISTFILES += \
    ../CMakeLists.txt

HEADERS += \
    ../include/maddrive_cv_tools/cvtrafficlightconfig.h
