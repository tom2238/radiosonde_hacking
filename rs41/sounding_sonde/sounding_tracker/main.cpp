#include "mainwindow.h"
#include <QApplication>
#include <QTranslator>
#include <QLibraryInfo>

/**
 * @brief main
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    QTranslator translator;
    if(translator.load("qt_" + QLocale::system().name(), QLibraryInfo::location(QLibraryInfo::TranslationsPath))) {
        // nacteni jazyka ok
        qDebug() << "Locale name, " << QLocale::system().name() << ", location, " << QLibraryInfo::location(QLibraryInfo::TranslationsPath);
        a.installTranslator(&translator);
    }
    MainWindow w;
    w.show();
    return a.exec();
}
