#include "mainwindow.h"
#include "ui_mainwindow.h"

/**
 * @brief MainWindow::MainWindow
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    // Setup
    ui->setupUi(this);
    // UI name
    this->setWindowTitle(APPLICATION_DISPLAY_NAME);

    // Load settings
    LoadSettings();

    // Set UI parts
    static QDoubleValidator posLatValid, posLonValid;
    posLatValid.setLocale(QLocale::system());
    posLonValid.setLocale(QLocale::system());
    posLatValid.setRange(-90.0,90.0,7);
    posLonValid.setRange(-180.0,180.0,7);
    posLatValid.setNotation(QDoubleValidator::StandardNotation);
    posLonValid.setNotation(QDoubleValidator::StandardNotation);
    ui->LE_GS_lat->setValidator(&posLatValid);
    ui->LE_GS_lon->setValidator(&posLonValid);
    packet_hex_document = QHexDocument::create(this);
    ui->HX_packet_view->setDocument(packet_hex_document);
    ui->LB_stat_sync->setAutoFillBackground(true);
    ui->LB_stat_crc->setAutoFillBackground(true);
    ui->LB_stat_valid->setAutoFillBackground(true);
    ui->LB_stat_net->setAutoFillBackground(true);

    // Label selectable
    ui->label_11->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_12->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_13->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_14->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_15->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_17->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_18->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_20->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_21->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_24->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_26->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->label_28->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_frequency->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_power->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_id->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_frame->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_voltage->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gpstime->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_temperature->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_lat->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_lon->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_alt->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_groundspeed->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_climbing->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_heading->setTextInteractionFlags(Qt::TextSelectableByMouse);
    ui->LB_rx_gps_numsv->setTextInteractionFlags(Qt::TextSelectableByMouse);

    // Create objects
    QList<double> pos;
    pos.append(QLocale().system().toDouble(ui->LE_GS_lat->text()));
    pos.append(QLocale().system().toDouble(ui->LE_GS_lon->text()));
    pos.append(STATION_POSITION_ALT);
    sondehub = new QSondeHub(this,ui->LE_callsign->text(),pos,ui->LE_GS_radioType->text(),ui->LE_GS_antenna->text(),ui->LE_GS_comment->text(),APPLICATION_NAME,APP_VERSION,2,20,5,false);

    // Audio list sources
    RefreshInputAudioDevices();

    // Connect all
    // Timers
    connect(&sync_timeout_timer,&QTimer::timeout,this,&MainWindow::sync_timeout);
    connect(&crc_timeout_timer,&QTimer::timeout,this,&MainWindow::crc_timeout);
    connect(&valid_timeout_timer,&QTimer::timeout,this,&MainWindow::valid_timeout);
    connect(&net_timeout_timer,&QTimer::timeout,this,&MainWindow::net_timeout);
    connect(&station_position_timer,&QTimer::timeout,this,&MainWindow::station_position_timeout);
    // Decoder
    connect(&frame_decoder,&QAMFrame::SyncReceived,this,&MainWindow::sync_detected);
    connect(&frame_decoder,&QAMFrame::CrcReceived,this,&MainWindow::crc_detected);
    connect(&frame_decoder,&QAMFrame::ValidFrameReceived,this,&MainWindow::valid_detected);
    connect(sondehub,&QSondeHub::NetworkReplyReceived,this,&MainWindow::net_reply_received);
    connect(&frame_decoder,&QAMFrame::PacketReceived,this,&MainWindow::packet_received);
    // Logging
    connect(sondehub, &QSondeHub::LogUI, this, &MainWindow::ConsoleLogSignal);

    // Sync timer stop
    sync_timeout_timer.stop();
    sync_timeout_timer.setSingleShot(true);

    // CRC timer stop
    crc_timeout_timer.stop();
    crc_timeout_timer.setSingleShot(true);

    // Valid data timer stop
    valid_timeout_timer.stop();
    valid_timeout_timer.setSingleShot(true);

    // Net upload success timer stop
    net_timeout_timer.stop();
    net_timeout_timer.setSingleShot(true);

    // Station position upload timer
    station_position_timer.stop();
    station_position_timer.setSingleShot(false);
    station_position_timer.setInterval(15*60*1000); // 15 minutes

    // Buttons config
    ui->PB_stop->setEnabled(false);
    ui->PB_start->setEnabled(true);

    // Get decoded packet and update UI
    UpdateSoundingUI(false);

    // Upload station position
    if(ui->CX_habhubEnable->isChecked()) {
        sondehub->StationPositionUpload();
        station_position_timer.start();
    }

    QString version(GIT_VERSION);
    QString buildDate = QStringLiteral(__DATE__) + QStringLiteral(" ") + QStringLiteral(__TIME__);
    qDebug() << "Ver:" << version << ", bd:" << buildDate;
    ConsoleLog("Start",MainWindow::LOG_INFO);
    ConsoleLog("App: " + QString(APPLICATION_NAME) + ", Ver: " + version + ", bd: " + buildDate, MainWindow::LOG_INFO);
}

/**
 * @brief MainWindow::~MainWindow
 */
MainWindow::~MainWindow() {
    // Save settings
    SaveSettings();
    // Delete UI
    delete ui;
    // Sonde hub
    delete sondehub;
}

/**
 * @brief MainWindow::on_PB_start_clicked
 */
void MainWindow::on_PB_start_clicked() {
    QVariant v = ui->CB_InputAudioDevice->currentData();
    QAudioDeviceInfo dev = v.value<QAudioDeviceInfo>();
    qDebug() << dev.supportedSampleTypes();

    QAudioFormat format;
    format.setSampleRate(48000);
    format.setChannelCount(1);
    format.setSampleType(QAudioFormat::SignedInt);
    format.setByteOrder(QAudioFormat::LittleEndian);
    format.setCodec("audio/pcm");
    format.setSampleSize(16);

    // Init decoder
    if(frame_decoder.Init(false,false,false,format,VAISALA_MODEM_BAUDRATE,VAISALA_MODEM_PACKET_SIZE,FRAME_MOD_NRZ,VAISALA_ECC_MODE)) {
        // Invalid config
        on_PB_stop_clicked();
        return;
    }

    if (mAudioIn) {
        delete mAudioIn;
    }
    mAudioIn = nullptr;
    mAudioIn = new QAudioInput(dev,format);
    mAudioIn->setVolume(1);
    mAudioIn->setNotifyInterval(50);

    connect(mAudioIn, &QAudioInput::notify,this, &MainWindow::processAudioIn);
    connect(mAudioIn, &QAudioInput::stateChanged,this, &MainWindow::stateChangeAudioIn);

    // Init frame and head buffer
    mInputBuffer.open(QBuffer::ReadWrite);
    mAudioIn->start(&mInputBuffer);

    // Buttons
    ui->PB_stop->setEnabled(true);
    ui->PB_start->setEnabled(false);
    // Setup
    ui->CB_InputAudioDevice->setEnabled(false);
    ui->LE_callsign->setEnabled(false);
    ui->LE_GS_lat->setEnabled(false);
    ui->LE_GS_lon->setEnabled(false);
    ui->LE_GS_antenna->setEnabled(false);
    ui->LE_GS_comment->setEnabled(false);
    ui->LE_GS_radioType->setEnabled(false);

    ConsoleLog("Start decoder success",MainWindow::LOG_INFO);
}

/**
 * @brief MainWindow::processAudioIn
 */
void MainWindow::processAudioIn() {
    mInputBuffer.seek(0);
    QByteArray ba = mInputBuffer.readAll();
    int sample_byte;

    int num_samples = ba.length() / 2;
    int b_pos = 0;
    for (int i = 0; i < num_samples; i ++) {
        int16_t s = 0;
        s = static_cast<int16_t>(ba.at(b_pos++)) & 0xFF;
        s |= static_cast<int16_t>(ba.at(b_pos++) & 0xFF) << 8;
        //qDebug() << QString::number(s);
        if (s != 0) {
            sample_byte = s;
        } else {
            sample_byte = 0;
        }
        frame_decoder.ReadAudioSample(sample_byte);
    }
    //qDebug() << "Audio notify size: " << mInputBuffer.size();

    mInputBuffer.buffer().clear();
    mInputBuffer.seek(0);
}

/**
 * @brief MainWindow::stateChangeAudioIn
 * @param s
 */
void MainWindow::stateChangeAudioIn(QAudio::State s) {
    //qDebug() << "State change: " << s;
}

/**
 * @brief MainWindow::on_PB_stop_clicked
 */
void MainWindow::on_PB_stop_clicked() {
    // Stop
    if (mAudioIn) {
        mAudioIn->stop();
    }
    mInputBuffer.close();
    // Button
    ui->PB_stop->setEnabled(false);
    ui->PB_start->setEnabled(true);
    // Setup
    ui->CB_InputAudioDevice->setEnabled(true);
    ui->LE_callsign->setEnabled(true);
    ui->LE_GS_lat->setEnabled(true);
    ui->LE_GS_lon->setEnabled(true);
    ui->LE_GS_antenna->setEnabled(true);
    ui->LE_GS_comment->setEnabled(true);
    ui->LE_GS_radioType->setEnabled(true);
    ConsoleLog("Stop decoder success",MainWindow::LOG_INFO);
}

/**
 * @brief MainWindow::RefreshInputAudioDevices
 */
void MainWindow::RefreshInputAudioDevices(void) {
    // List devices
    QList<QAudioDeviceInfo> inputDevices = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);
    // Clear box
    ui->CB_InputAudioDevice->clear();
    // Insert items
    for (QAudioDeviceInfo d : inputDevices) {
        ui->CB_InputAudioDevice->addItem(d.deviceName(),QVariant::fromValue(d));
    }
}

/**
 * @brief MainWindow::LoadSettings
 */
void MainWindow::LoadSettings(void) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    // Station
    ui->LE_callsign->setText(application_setting.value("Station/Callsign","N0CALL").toString());
    ui->LE_GS_antenna->setText(application_setting.value("Station/Antenna","Ground plane").toString());
    ui->LE_GS_lat->setText(application_setting.value("Station/Latitude","17.0000").toString());
    ui->LE_GS_lon->setText(application_setting.value("Station/Longitude","49.0000").toString());
    ui->LE_GS_comment->setText(application_setting.value("Station/Comment","Receiver test").toString());
    ui->LE_GS_radioType->setText(application_setting.value("Station/Radio","RTL-SDR").toString());
    // Feed
    bool aprs_en = application_setting.value("Feed/APRS","false").toBool();
    bool habhub_en = application_setting.value("Feed/SondeHub","false").toBool();
    if(aprs_en) {
        ui->CX_aprsEnable->setCheckState(Qt::CheckState::Checked);
    } else {
        ui->CX_aprsEnable->setCheckState(Qt::CheckState::Unchecked);
    }
    if(habhub_en) {
        ui->CX_habhubEnable->setCheckState(Qt::CheckState::Checked);
    } else {
        ui->CX_habhubEnable->setCheckState(Qt::CheckState::Unchecked);
    }
}

/**
 * @brief MainWindow::SaveSettings
 */
void MainWindow::SaveSettings(void) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    // Station
    application_setting.setValue("Station/Callsign",ui->LE_callsign->text());
    application_setting.setValue("Station/Antenna",ui->LE_GS_antenna->text());
    application_setting.setValue("Station/Latitude",ui->LE_GS_lat->text());
    application_setting.setValue("Station/Longitude",ui->LE_GS_lon->text());
    application_setting.setValue("Station/Comment",ui->LE_GS_comment->text());
    application_setting.setValue("Station/Radio",ui->LE_GS_radioType->text());
    // Feed
    application_setting.setValue("Feed/APRS",ui->CX_aprsEnable->isChecked());
    application_setting.setValue("Feed/SondeHub",ui->CX_habhubEnable->isChecked());
}

/**
 * @brief MainWindow::sync_timeout
 */
void MainWindow::sync_timeout(void) {
    QPalette pal = ui->LB_stat_sync->palette();
    pal.setColor(QPalette::Window, QColor(Qt::transparent));
    ui->LB_stat_sync->setPalette(pal);
}

/**
 * @brief MainWindow::sync_detected
 */
void MainWindow::sync_detected(void) {
    // Change background color
    QPalette pal = ui->LB_stat_sync->palette();
    pal.setColor(QPalette::Window, QColor(Qt::green));
    ui->LB_stat_sync->setPalette(pal);
    // Start timer, after hit clear background
    sync_timeout_timer.start(100);
}

/**
 * @brief MainWindow::crc_timeout
 */
void MainWindow::crc_timeout(void) {
    // Change background color
    QPalette pal = ui->LB_stat_crc->palette();
    pal.setColor(QPalette::Window, QColor(Qt::transparent));
    ui->LB_stat_crc->setPalette(pal);
}

/**
 * @brief MainWindow::crc_detected
 * @param state
 */
void MainWindow::crc_detected(bool state) {
    // Change background color
    QPalette pal = ui->LB_stat_crc->palette();
    if(state) {
        pal.setColor(QPalette::Window, QColor(Qt::green));
    } else {
        pal.setColor(QPalette::Window, QColor(Qt::red));
    }
    ui->LB_stat_crc->setPalette(pal);
    // Start timer, after hit clear background
    crc_timeout_timer.start(100);
}

/**
 * @brief MainWindow::valid_timeout
 */
void MainWindow::valid_timeout(void) {
    // Change background color
    QPalette pal = ui->LB_stat_valid->palette();
    pal.setColor(QPalette::Window, QColor(Qt::transparent));
    ui->LB_stat_valid->setPalette(pal);
}

void MainWindow::valid_detected(bool state) {
    // Change background color
    QPalette pal = ui->LB_stat_valid->palette();
    if(state) {
        pal.setColor(QPalette::Window, QColor(Qt::green));
    } else {
        pal.setColor(QPalette::Window, QColor(Qt::red));
    }
    ui->LB_stat_valid->setPalette(pal);
    // Start timer, after hit clear background
    valid_timeout_timer.start(100);
}

/**
 * @brief MainWindow::net_timeout
 */
void MainWindow::net_timeout(void) {
    // Change background color
    QPalette pal = ui->LB_stat_net->palette();
    pal.setColor(QPalette::Window, QColor(Qt::transparent));
    ui->LB_stat_net->setPalette(pal);
}

/**
 * @brief MainWindow::net_reply_received
 * @param httpCode
 */
void MainWindow::net_reply_received(int httpCode) {
    // Change background color
    QPalette pal = ui->LB_stat_net->palette();
    if((httpCode >= 100) && (httpCode <= 199)) {
        pal.setColor(QPalette::Window, QColor(Qt::blue));
    } else if(httpCode == 200) {
        pal.setColor(QPalette::Window, QColor(Qt::green));
    } else if((httpCode >= 300) && (httpCode <= 399)) {
        pal.setColor(QPalette::Window, QColor(Qt::yellow));
    } else if((httpCode >= 400) && (httpCode <= 499)) {
        pal.setColor(QPalette::Window, QColor(Qt::magenta));
    } else {
        pal.setColor(QPalette::Window, QColor(Qt::red));
    }
    ui->LB_stat_net->setPalette(pal);
    // Start timer, after hit clear background
    net_timeout_timer.start(100);
}

/**
 * @brief MainWindow::station_position_timeout
 */
void MainWindow::station_position_timeout(void) {
    // Upload station position
    if(ui->CX_habhubEnable->isChecked()) {
        sondehub->StationPositionUpload();
    }
}

/**
 * @brief MainWindow::packet_received
 */
void MainWindow::packet_received(void) {
    FrameData frm_last = frame_decoder.GetLastFrame();
    QByteArray packet_bytes;
    for(int i=0;i<frm_last.length;i++) {
        packet_bytes.append(static_cast<char>(frm_last.value[i]));
    }
    // Set RAW data
    packet_hex_document->remove(0,packet_hex_document->length());
    packet_hex_document->replace(0,packet_bytes);
    // Decode packet
    frame_decoder.DecodeFrame(&frm_last);
    // Get decoded packet and update UI
    UpdateSoundingUI(true);
}

/**
 * @brief MainWindow::UpdateSoundingUI
 */
void MainWindow::UpdateSoundingUI(bool net_upload_enable) {
    QMap<QString, QString> rec_data;
    rec_data = frame_decoder.GetDecodedFrame();
    QString gps_year = rec_data.value("GPS_YEAR");
    QString gps_month = rec_data.value("GPS_MONTH");
    QString gps_day = rec_data.value("GPS_DAY");
    QString gps_hour = rec_data.value("GPS_HOUR");
    QString gps_min = rec_data.value("GPS_MINUTE");
    QString gps_sec = rec_data.value("GPS_SECOND");

    // Set UI
    ui->LB_rx_gpstime->setText(gps_year + "/" + gps_month + "/" + gps_day + " " + gps_hour + ":" + gps_min + ":" + gps_sec);
    ui->LB_rx_gps_lat->setText(rec_data.value("GPS_LAT"));
    ui->LB_rx_gps_lon->setText(rec_data.value("GPS_LON"));
    ui->LB_rx_gps_alt->setText(rec_data.value("GPS_ALT") + " m");
    ui->LB_rx_gps_climbing->setText(rec_data.value("GPS_CLIMBING") + " m/s");
    ui->LB_rx_gps_numsv->setText(rec_data.value("GPS_NUMSV"));
    ui->LB_rx_gps_groundspeed->setText(rec_data.value("GPS_GROUNDSPEED") + " m/s");
    ui->LB_rx_gps_heading->setText(rec_data.value("GPS_HEADING") + " °");
    ui->LB_rx_temperature->setText(rec_data.value("TEMPERATURE_MAIN") + " °C");
    ui->LB_rx_id->setText(rec_data.value("SONDE_ID"));
    ui->LB_rx_frame->setText(rec_data.value("FRAME_COUNT"));
    ui->LB_rx_voltage->setText(rec_data.value("ONBOARD_VOLTAGE") + " V");
    ui->LB_rx_frequency->setText(rec_data.value("TX_FREQUENCY") + " MHz");
    ui->LB_rx_power->setText(rec_data.value("TX_POWER") + " dBm");

    // Send data to sondehub
    if(net_upload_enable) {
        if(ui->CX_habhubEnable->isChecked()) {
            QDateTime dt = QDateTime::currentDateTime();
            sondehub->TelemetryUpload(rec_data, dt);
        }
    }

}

/**
 * @brief MainWindow::ConsoleLog
 * @param string
 * @param type_e
 */
void MainWindow::ConsoleLog(QString string, MainWindow::LogType_e type_e) {
    // Zmena ukazatele na konec
    QTextCursor TE_cursor(ui->TE_logArea->textCursor());
    TE_cursor.movePosition(QTextCursor::End);
    ui->TE_logArea->setTextCursor(TE_cursor);
    // Aktualni cas
    QDateTime dateTime = dateTime.currentDateTime();
    //qDebug() << dateTime.toString("HH:mm:ss");
    QString logStr;
    switch (type_e) {
    case LOG_INFO:
        logStr = " INF ";
        break;
    case LOG_DEBUG:
        logStr = " DBG ";
        break;
    case LOG_WARNING:
        logStr = " WRN ";
        break;
    case LOG_CRITICAL:
        logStr = " CRT ";
        break;
    }
    // Vypis na konzoli
    ui->TE_logArea->insertPlainText(dateTime.toString("HH:mm:ss") + logStr + string + "\n");
    // Posun textu na konec
    //ui->TE_logArea->verticalScrollBar()->setValue(ui->TE_logArea->verticalScrollBar()->maximum());
}

/**
 * @brief MainWindow::ConsoleLogSignal
 * @param string
 * @param type
 */
void MainWindow::ConsoleLogSignal(QString string, qint8 type) {
    MainWindow::LogType_e log;
    switch(type) {
    case 0:
        log = LOG_INFO;
        break;
    case 2:
        log = LOG_WARNING;
        break;
    case 3:
        log = LOG_CRITICAL;
        break;
    default:
        log = LOG_DEBUG;
        break;
    }

    ConsoleLog(string,log);
}

/**
 * @brief MainWindow::on_LE_callsign_textChanged
 * @param arg1
 */
void MainWindow::on_LE_callsign_textChanged(const QString &arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Station/Callsign",arg1);
    if(sondehub) {
        sondehub->UpdateCallsign(arg1);
    }

}

/**
 * @brief MainWindow::on_LE_GS_comment_textChanged
 * @param arg1
 */
void MainWindow::on_LE_GS_comment_textChanged(const QString &arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Station/Comment",arg1);
    if(sondehub) {
        sondehub->UpdateEmail(arg1);
    }
}

/**
 * @brief MainWindow::on_LE_GS_lat_textChanged
 * @param arg1
 */
void MainWindow::on_LE_GS_lat_textChanged(const QString &arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Station/Latitude",arg1);
    if(sondehub) {
        sondehub->UpdateStationPosition(QLocale().system().toDouble(ui->LE_GS_lat->text()),QLocale().system().toDouble(ui->LE_GS_lon->text()),STATION_POSITION_ALT);
    }
}

/**
 * @brief MainWindow::on_LE_GS_lon_textChanged
 * @param arg1
 */
void MainWindow::on_LE_GS_lon_textChanged(const QString &arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Station/Longitude",arg1);
    if(sondehub) {
        sondehub->UpdateStationPosition(QLocale().system().toDouble(ui->LE_GS_lat->text()),QLocale().system().toDouble(ui->LE_GS_lon->text()),STATION_POSITION_ALT);
    }
}

/**
 * @brief MainWindow::on_LE_GS_antenna_textChanged
 * @param arg1
 */
void MainWindow::on_LE_GS_antenna_textChanged(const QString &arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Station/Antenna",arg1);
    if(sondehub) {
        sondehub->UpdateAntenna(arg1);
    }
}

/**
 * @brief MainWindow::on_LE_GS_radioType_textChanged
 * @param arg1
 */
void MainWindow::on_LE_GS_radioType_textChanged(const QString &arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Station/Radio",arg1);
    if(sondehub) {
        sondehub->UpdateRadio(arg1);
    }
}

/**
 * @brief MainWindow::on_CX_habhubEnable_stateChanged
 * @param arg1
 */
void MainWindow::on_CX_habhubEnable_stateChanged(int arg1) {
    QSettings application_setting(ORGANIZATION_NAME, APPLICATION_NAME);
    application_setting.setValue("Feed/SondeHub",ui->CX_habhubEnable->isChecked());
    if(arg1 == Qt::Unchecked) {
        // Unchecked
        station_position_timer.stop();
    } else {
        // Checked or partialy checked
        station_position_timer.start();
        if(sondehub) {
            sondehub->StationPositionUpload();
        }
    }
}
