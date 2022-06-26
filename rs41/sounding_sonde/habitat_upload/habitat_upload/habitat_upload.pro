TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        base64.c \
        main.c \
        sha256.c

HEADERS += \
    base64.h \
    sha256.h

unix|win32: LIBS += -lcurl
