QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += sdk_no_version_check

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    SeamCarve.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    SeamCarve.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += /usr/local/Cellar/opencv/4.5.0_5/include/opencv4/opencv2/
INCLUDEPATH += /usr/local/Cellar/opencv/4.5.0_5/include/
INCLUDEPATH += /usr/local/Cellar/opencv/4.5.0_5/include/opencv4/
LIBS += -L/usr/local/Cellar/opencv/4.5.0_5/lib/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

DISTFILES += \
    data/lotus.jpg \
    data/pliers.jpg \
    data/remove.jpg \
    data/remove_mask.jpg \
    data/remove_result.jpg \
    data/test.jpg
