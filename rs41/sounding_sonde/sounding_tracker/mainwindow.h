#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSettings>
#include <QDebug>
#include <QtMultimedia/QAudioInput>
#include <QBuffer>
#include <QtMath>
#include <QTimer>
#include <QMap>
#include "QHexView/qhexview.h"
#include "qamframe.h"
#include "qsondehub.h"

#define ORGANIZATION_NAME "tom2238"
#define ORGANIZATION_DOMAIN "github.com/tom2238"
#define APPLICATION_NAME "sounding_tracker"
#define APPLICATION_VERSION "0.1"
#define APPLICATION_DISPLAY_NAME "Sounding tracker"
#define VAISALA_MODEM_BAUDRATE 4800
#define VAISALA_MODEM_PACKET_SIZE 62
#define VAISALA_ECC_MODE 0

// Progress bar limit
#define UI_PG_SYNC_MAX_LIMIT 64

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
    void sync_timeout();
    void sync_detected();
    void packet_received();

private:
    // vars
    Ui::MainWindow *ui;
    QAudioInput *mAudioIn = nullptr;
    QBuffer  mInputBuffer;
    QAMFrame frame_decoder;
    QTimer sync_timeout_timer;
    bool sync_detected_pg;
    QHexDocument *packet_hex_document;
    // funcs
    void RefreshInputAudioDevices(void);
    void LoadSettings(void);
    void SaveSettings(void);
    void UpdateSoundingUI(void);
};
#endif // MAINWINDOW_H
