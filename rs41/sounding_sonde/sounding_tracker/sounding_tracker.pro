QT       += core gui multimedia network printsupport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

GIT_VERSION = $$system(git --git-dir $$shell_quote($$PWD/../../../.git) --work-tree $$shell_quote($$PWD/../../../) describe --always --tags)
DEFINES += GIT_VERSION=$$shell_quote(\"$$GIT_VERSION\")
GIT_COMMIT_COUNT = $$system(git rev-list --count --all)
DEFINES += GIT_COMMIT_COUNT=$$shell_quote(\"$$GIT_COMMIT_COUNT\")

VERSION = 0.1.0.$$GIT_COMMIT_COUNT
DEFINES += APP_VERSION=\\\"$$VERSION\\\"

INCLUDEPATH += $$PWD/include
include(QHexView.pri)

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    amframe.cpp \
    main.cpp \
    mainwindow.cpp \
    qamframe.cpp \
    qcustomplot.cpp \
    qsondehub.cpp \
    ssfrs.cpp

HEADERS += \
    amframe.h \
    mainwindow.h \
    qamframe.h \
    qcustomplot.h \
    qsondehub.h \
    ssfrs.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
