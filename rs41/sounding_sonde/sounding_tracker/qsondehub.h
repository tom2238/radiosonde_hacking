#ifndef QSONDEHUB_H
#define QSONDEHUB_H

#include <QObject>
#include <QDebug>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrl>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QByteArray>
#include <QDateTime>

// SondeHub API endpoint http or https
#define SONDEHUB_URL QString("https://api.v2.sondehub.org/amateur/telemetry")
#define SONDEHUB_STATION_POSITION_URL QString("https://api.v2.sondehub.org/amateur/listeners")

class QSondeHub : public QObject
{
    Q_OBJECT

public:
    explicit QSondeHub(QObject *parent = nullptr,
                       QString uploader_callsign_p = "N0CALL",
                       QList<double> uploader_position_p = QList<double>(),
                       QString uploader_radio_p = "SDR",
                       QString uploader_antenna_p = "GP",
                       QString uploader_email_p = "nocall@mail.qrz",
                       QString software_name_p = "QSondeHub",
                       QString software_version_p = "0.0.1",
                       int upload_rate_p = 2,
                       int upload_timeout_p = 20,
                       int upload_retries_p = 5,
                       bool  developer_mode_p = false
            );
    ~QSondeHub();
    void StationPositionUpload(void);
    void TelemetryUpload(QMap<QString, QString> frame, QDateTime timeReceived);
    void UpdateStationPosition(double lat, double lon, double alt);
    void UpdateCallsign(QString call);
    void UpdateAntenna(QString antenna);
    void UpdateRadio(QString radio);
    void UpdateEmail(QString email);

signals:
    void NetworkReplyReceived(int);
    void LogUI(QString, qint8);
public slots:
    void replyFinished (QNetworkReply *reply);

private:
    QString uploader_callsign;
    QList<double> uploader_position;
    QString uploader_radio;
    QString uploader_antenna;
    QString uploader_email;
    QString software_name;
    QString software_version;
    int upload_rate;
    int upload_timeout;
    int upload_retries;
    bool developer_mode;
    QNetworkAccessManager *sondehub_netman;

};

#endif // QSONDEHUB_H
