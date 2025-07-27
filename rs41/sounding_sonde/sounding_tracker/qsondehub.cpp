/*
 * https://github.com/projecthorus/radiosonde_auto_rx/blob/master/auto_rx/autorx/sondehub.py
 * https://github.com/projecthorus/pysondehub/blob/main/sondehub/amateur.py
 * https://repo.radio/VE7UWU/sdrangel/src/branch/master/sdrbase/util/sondehub.cpp
 *
 * Uploads telemetry to the 'new' SondeHub ElasticSearch cluster,
 * in the new 'universal' format descried here:
 * https://github.com/projecthorus/radiosonde_auto_rx/wiki/Suggested-Universal-Sonde-Telemetry-Format
 */
#include "qsondehub.h"

/**
 * @brief QSondeHub::QSondeHub
 * @param parent
 * @param uploader_callsign_p  Callsign/name of the uploader
 * @param uploader_position_p Uploader position as [lat, lon, alt]
 * @param uploader_radio_p  Information on the Uploader's radio.
 * @param uploader_antenna_p  Information on the Uploader's antenna
 * @param software_name_p Software name information. If this is provided, software_version must also be provided.
 * @param software_version_p Software version number, e.g. "0.1.2"
 * @param upload_rate_p How often to upload batches of data.
 * @param upload_timeout_p Upload timeout (seconds)
 * @param upload_retries_p Upload retries
 * @param developer_mode_p If set to true, packets will be discarded when they enter the Sondehub DB.
 */
QSondeHub::QSondeHub(QObject *parent,
                     QString uploader_callsign_p,
                     QList<double> uploader_position_p,
                     QString uploader_radio_p,
                     QString uploader_antenna_p,
                     QString uploader_email_p,
                     QString software_name_p,
                     QString software_version_p,
                     int upload_rate_p,
                     int upload_timeout_p,
                     int upload_retries_p,
                     bool  developer_mode_p) : QObject(parent) {

    // Set all
    uploader_callsign = uploader_callsign_p;
    uploader_position = uploader_position_p;
    uploader_radio = uploader_radio_p;
    uploader_antenna = uploader_antenna_p;
    uploader_email = uploader_email_p;
    software_name = software_name_p;
    software_version = software_version_p;
    upload_rate = upload_rate_p;
    upload_timeout = upload_timeout_p;
    upload_retries = upload_retries_p;
    developer_mode = developer_mode_p;

    // Check position
    if(uploader_position_p.size() == 3) {
        uploader_position = uploader_position_p;
    } else {
        // Error
        qDebug() << "Invalid uploader position supplied, must be a list with 3 (lat, lon, alt) elements.";
    }

    // New network manager
    sondehub_netman = new QNetworkAccessManager(this);
    connect(sondehub_netman, SIGNAL(finished(QNetworkReply*)), this, SLOT(replyFinished(QNetworkReply*)));
    return;
}

/**
 * @brief QSondeHub::~QSondeHub
 */
QSondeHub::~QSondeHub() {
    delete sondehub_netman;
    return;
}

/**
 * @brief QSondeHub::StationPositionUpload
 */
void QSondeHub::StationPositionUpload(void) {
/*
 * Upload a station position packet to SondeHub.
 * Upload station position information to the SondeHub-Amateur Database.
 * This is distinct from the station information sent along with telemetry,
 * in that it results in a station icon being shown on the tracker map for
 * ~12 hours after the upload.
 *
 * If 'mobile' is set to True, then the station will appear as a chase-car
 * instead. In this case, positions should be uploaded at much more regular
 * intervals to reflect the movement of the chase car.
 *
 * This uses the PUT /amateur/listeners API described here:
 * https://github.com/projecthorus/sondehub-infra/wiki/API-(Beta)
 */

    // Check position range
    if(uploader_position.size() != 3) {
        qDebug() << "Invalid uploader position supplied, must be a list with 3 (lat, lon, alt) elements.";
    }

    // Position
    QJsonArray position {
            QString::number(uploader_position.at(0),'f',7), QString::number(uploader_position.at(1),'f',7), QString::number(uploader_position.at(2),'f',1)
     };

    QJsonObject obj {
           {"software_name", software_name},
           {"software_version", software_version},
           {"uploader_callsign", uploader_callsign},
           {"uploader_position", position},
           {"uploader_radio", uploader_radio},
           {"uploader_antenna", uploader_antenna},
           {"uploader_contact_email", uploader_email},
           {"mobile", false}
       };

    QJsonDocument json_position_doc(obj);
    QByteArray json_post_data = json_position_doc.toJson(QJsonDocument::Compact);
    //qDebug().noquote() << json_post_data;

    QUrl url(SONDEHUB_STATION_POSITION_URL);
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    request.setHeader(QNetworkRequest::ContentLengthHeader, QByteArray::number(json_post_data.size()));
    request.setHeader(QNetworkRequest::UserAgentHeader, software_name + "-V:" + software_version);

    sondehub_netman->put(request, json_post_data);
    return;
}

/**
 * @brief QSondeHub::TelemetryUpload
 * @param frame
 */
void QSondeHub::TelemetryUpload(QMap<QString, QString> frame, QDateTime timeReceived) {
    //qDebug() << "Valid frame" << frame.value("VALID").toUInt();
    // We have invalid data then return
    if(frame.value("VALID").toUInt() == 0) {
        return;
    }
    // If valid continue

    QJsonArray uploaderPos {
        QString::number(uploader_position.at(0), 'f', 7).toDouble(),
        QString::number(uploader_position.at(1), 'f', 7).toDouble(),
        QString::number(uploader_position.at(2), 'f', 1).toDouble()
    };

    QJsonObject obj {
        {"software_name", software_name},
        {"software_version", software_version},
        {"uploader_callsign", uploader_callsign},
        {"time_received", timeReceived.toUTC().toString("yyyy-MM-ddTHH:mm:ss.zzz000Z")},
        {"uploader_position", uploaderPos}
    };

    obj.insert("frame", frame.value("FRAME_COUNT").toInt());
    obj.insert("payload_callsign", frame.value("SONDE_ID"));
    obj.insert("batt", frame.value("ONBOARD_VOLTAGE").toFloat());
    obj.insert("ext_temperature", frame.value("TEMPERATURE_MAIN").toFloat());

    // For develop only
    if(developer_mode) {
        obj.insert("dev","true");
    }

    QDateTime gpsDateTime;
    QDate gpsDate;
    QTime gpsTime;
    gpsDate.setDate(frame.value("GPS_YEAR").toInt(),frame.value("GPS_MONTH").toInt(),frame.value("GPS_DAY").toInt());
    gpsTime.setHMS(frame.value("GPS_HOUR").toInt(),frame.value("GPS_MINUTE").toInt(),frame.value("GPS_SECOND").toInt());
    gpsDateTime.setDate(gpsDate);
    gpsDateTime.setTime(gpsTime);
    obj.insert("datetime", gpsDateTime.toUTC().addSecs(18).toString("yyyy-MM-ddTHH:mm:ss.zzz000Z")); // +18 adjusts UTC to GPS time

    obj.insert("lat", frame.value("GPS_LAT").toFloat());
    obj.insert("lon", frame.value("GPS_LON").toFloat());
    obj.insert("alt", frame.value("GPS_ALT").toFloat());
    obj.insert("vel_h", frame.value("GPS_GROUNDSPEED"));
    obj.insert("vel_v", frame.value("GPS_CLIMBING"));
    obj.insert("heading", frame.value("GPS_HEADING").toFloat());
    obj.insert("sats", frame.value("GPS_NUMSV").toInt());

    obj.insert("tx_frequency", frame.value("TX_FREQUENCY").toFloat());
    obj.insert("modulation","FM");
    obj.insert("baud_rate","4800");

    QJsonArray payloads {
        obj
    };

    QJsonDocument doc(payloads);
    QByteArray data = doc.toJson(QJsonDocument::Compact);
    //qDebug().noquote() << data;

    QUrl url(QString(SONDEHUB_URL));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    request.setHeader(QNetworkRequest::UserAgentHeader, software_name + "-V:" + software_version);
    request.setRawHeader("Date", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs).toLatin1());
    request.setHeader(QNetworkRequest::ContentLengthHeader, QByteArray::number(data.size()));
    request.setRawHeader("accept","text/plain");

    sondehub_netman->put(request, data);
    return;
}

/**
 * @brief QSondeHub::replyFinished
 * @param reply
 */
void QSondeHub::replyFinished (QNetworkReply *reply) {
    int httpStatusCode;
    if(reply) {
        httpStatusCode = reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
        if(reply->error()) {
            qDebug() << "ERROR!";
            qDebug() << reply->errorString();
            qDebug() << httpStatusCode;
            qDebug().noquote() << reply->readAll();
            QString erro;
            erro.append("Network error. ");
            erro.append(reply->errorString());
            erro.append(httpStatusCode);
            erro.append(reply->readAll());
            emit LogUI(erro,2);
        } else {
            QByteArray bytes = reply->readAll();
            QJsonDocument document = QJsonDocument::fromJson(bytes);
            if (document.isObject()) {
                QJsonObject obj = document.object();
                if (obj.contains(QStringLiteral("message"))) {
                    QString message = obj.value(QStringLiteral("message")).toString();
                    qWarning() << "SondeHub message:" << message;
                    emit LogUI("SondeHub message:" + message,2);
                }
                if (obj.contains(QStringLiteral("errors")))   {
                    QJsonArray errors = obj.value(QStringLiteral("errors")).toArray();
                    for (auto errorObjRef : errors)   {
                        QJsonObject errorObj = errorObjRef.toObject();
                        if (errorObj.contains(QStringLiteral("error_message")))   {
                            QString errorMessage = errorObj.value(QStringLiteral("error_message")).toString();
                            qWarning() << "SondeHub error:" << errorMessage;
                            emit LogUI("SondeHub error:" + errorMessage,2);
                            if (errorObj.contains(QStringLiteral("payload")))   {
                                QJsonObject payload = errorObj.value(QStringLiteral("payload")).toObject();
                                qWarning() << "SondeHub error:" << QJsonDocument(payload);
                                emit LogUI("SondeHub error:" + QJsonDocument(payload).toJson(),2);
                            }
                        } else {
                            qWarning() << "SondeHub error:" << QJsonDocument(errorObj);
                            emit LogUI("SondeHub error:" + QJsonDocument(errorObj).toJson(),2);
                        }
                    }
                }
            } else if (document.isArray()) {
                QJsonArray array = document.array();
                for (auto arrayRef : array) {
                    qDebug() << "SondeHub::handleReply:" << bytes;
                }
            } else {
                qDebug() << "SondeHub::handleReply:" << bytes;
            }

            qDebug() << reply->header(QNetworkRequest::ContentTypeHeader).toString();
            qDebug() << reply->header(QNetworkRequest::LastModifiedHeader).toDateTime().toString();
            qDebug() << reply->header(QNetworkRequest::ContentLengthHeader).toULongLong();
            qDebug() << httpStatusCode;
            qDebug() << reply->attribute(QNetworkRequest::HttpReasonPhraseAttribute).toString();
            qDebug().noquote() << reply->readAll();
        }
        emit NetworkReplyReceived(httpStatusCode);
        reply->deleteLater();
    } else {
        qDebug() << "SondeHub::handleReply: reply is null";
    }
    return;
}

/**
 * @brief QSondeHub::UpdateStationPosition  Update the internal station position record. Used when determining the station position by GPSD
 * @param lat
 * @param lon
 * @param alt
 */
void QSondeHub::UpdateStationPosition(double lat, double lon, double alt) {
    // Clear and set pointer to zero
    uploader_position.clear();
    // Add new data
    uploader_position.append(lat);
    uploader_position.append(lon);
    uploader_position.append(alt);
    return;
}

/**
 * @brief QSondeHub::UpdateCallsign
 * @param call
 */
void QSondeHub::UpdateCallsign(QString call) {
    uploader_callsign = call;
    return;
}

/**
 * @brief QSondeHub::UpdateAntenna
 * @param antenna
 */
void QSondeHub::UpdateAntenna(QString antenna) {
    uploader_antenna = antenna;
    return;
}

/**
 * @brief QSondeHub::UpdateRadio
 * @param radio
 */
void QSondeHub::UpdateRadio(QString radio) {
    uploader_radio = radio;
    return;
}

/**
 * @brief QSondeHub::UpdateEmail
 * @param email
 */
void QSondeHub::UpdateEmail(QString email) {
    uploader_email = email;
    return;
}
