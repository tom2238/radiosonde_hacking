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
#include <QLocale>
#include <QDoubleValidator>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include "QHexView/qhexview.h"
#include "qamframe.h"
#include "qsondehub.h"
#include "qcustomplot.h"

#define ORGANIZATION_NAME "tom2238"
#define ORGANIZATION_DOMAIN "github.com/tom2238"
#define APPLICATION_NAME "sounding_tracker"
#define APPLICATION_VERSION "0.1.0.0"
#define APPLICATION_DISPLAY_NAME "Sounding tracker"
#define VAISALA_MODEM_BAUDRATE 4800
#define VAISALA_MODEM_PACKET_SIZE 62
#define VAISALA_ECC_MODE 0
#define STATION_POSITION_ALT 250.0

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
    typedef enum {
        LOG_INFO,
        LOG_DEBUG,
        LOG_WARNING,
        LOG_CRITICAL
    }LogType_e;
    Q_ENUM(LogType_e)

private slots:
    // Audio
    void processAudioIn();
    void stateChangeAudioIn(QAudio::State s);
    // Buttons
    void on_PB_start_clicked();
    void on_PB_stop_clicked();
    // Indicators
    void sync_timeout();
    void sync_detected();
    void crc_timeout();
    void crc_detected(bool state);
    void valid_timeout();
    void valid_detected(bool state);
    void net_timeout();
    void net_reply_received(int httpCode);
    void station_position_timeout();
    // Packet
    void packet_received();
    // Logging
    void ConsoleLogSignal(QString, qint8);

    void on_LE_callsign_textChanged(const QString &arg1);

    void on_LE_GS_comment_textChanged(const QString &arg1);

    void on_LE_GS_lat_textChanged(const QString &arg1);

    void on_LE_GS_lon_textChanged(const QString &arg1);

    void on_LE_GS_antenna_textChanged(const QString &arg1);

    void on_LE_GS_radioType_textChanged(const QString &arg1);

    void on_CX_habhubEnable_stateChanged(int arg1);

    void on_PB_dirSelect_clicked();

    void on_CX_logFileEnable_stateChanged(int arg1);

private:
    // vars
    Ui::MainWindow *ui = nullptr;
    QAudioInput *mAudioIn = nullptr;
    QBuffer  mInputBuffer;
    QAMFrame frame_decoder;
    QTimer sync_timeout_timer;
    QTimer crc_timeout_timer;
    QTimer valid_timeout_timer;
    QTimer net_timeout_timer;
    QTimer station_position_timer;
    QHexDocument *packet_hex_document = nullptr;
    QSondeHub *sondehub = nullptr;
    QFile CsvLogFile;
    // funcs
    void RefreshInputAudioDevices(void);
    void LoadSettings(void);
    void SaveSettings(void);
    void UpdateSoundingUI(bool net_upload_enable);
    void ConsoleLog(QString string, MainWindow::LogType_e type_e);
    bool CreateCsvLogFile(const QString &filename);
    bool CloseCsvLogFile(void);
    bool WriteCsvLogEvent(QMap<QString, QString> frame);
};
#endif // MAINWINDOW_H
