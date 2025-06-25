/*
 * https://github.com/projecthorus/radiosonde_auto_rx/blob/master/auto_rx/autorx/sondehub.py
 * https://github.com/projecthorus/pysondehub/blob/main/sondehub/amateur.py
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
    // Build json
    QJsonObject json_position;
    json_position["software_name"] = software_name;
    json_position["software_version"] = software_version;

    QJsonDocument json_position_doc(json_position);
    qDebug() << json_position_doc.toJson();

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
