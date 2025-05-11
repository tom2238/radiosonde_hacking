#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QDebug>
#include <QtMultimedia/QAudioInput>
#include <QBuffer>
#include <QtMath>
#include <QTimer>

#define ORGANIZATION_NAME "tom2238"
#define ORGANIZATION_DOMAIN "github.com/tom2238"
#define APPLICATION_NAME "sounding_tracker"
#define APPLICATION_VERSION "0.1"
#define APPLICATION_DISPLAY_NAME "Sounding tracker"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void processAudioIn();
    void stateChangeAudioIn(QAudio::State s);
    // Buttons
    void on_PB_start_clicked();
    void on_PB_stop_clicked();

private:
    // vars
    Ui::MainWindow *ui;
    QAudioInput *mAudioIn = nullptr;
    QBuffer  mInputBuffer;
    // funcs
    void RefreshInputAudioDevices(void);
    void LoadSettings(void);
    void SaveSettings(void);
};
#endif // MAINWINDOW_H
