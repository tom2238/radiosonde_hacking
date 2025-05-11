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
    // Setp
    ui->setupUi(this);
    // UI name
    this->setWindowTitle(APPLICATION_DISPLAY_NAME);

    // Audio list sources
    RefreshInputAudioDevices();

    // Connect all

    // Buttons config
    ui->PB_stop->setEnabled(false);
    ui->PB_start->setEnabled(true);

    // Load settings
    LoadSettings();
}

/**
 * @brief MainWindow::~MainWindow
 */
MainWindow::~MainWindow() {
    // Save settings
    SaveSettings();
    // Delete UI
    delete ui;
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

    if (mAudioIn) {
        delete mAudioIn;
    }
    mAudioIn = nullptr;
    mAudioIn = new QAudioInput(dev,format);
    mAudioIn->setVolume(1);
    mAudioIn->setNotifyInterval(50);

    connect(mAudioIn, &QAudioInput::notify,this, &MainWindow::processAudioIn);
    connect(mAudioIn, &QAudioInput::stateChanged,this, &MainWindow::stateChangeAudioIn);

    mInputBuffer.open(QBuffer::ReadWrite);
    mAudioIn->start(&mInputBuffer);
    // Start
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
}

/**
 * @brief MainWindow::processAudioIn
 */
void MainWindow::processAudioIn() {
    mInputBuffer.seek(0);
    QByteArray ba = mInputBuffer.readAll();


    int num_samples = ba.length() / 2;
    int b_pos = 0;
    for (int i = 0; i < num_samples; i ++) {
        int16_t s = 0;
        s = static_cast<int16_t>(ba.at(b_pos++)) & 0xFF;
        s |= static_cast<int16_t>(ba.at(b_pos++) & 0xFF) << 8;
        //qDebug() << QString::number(s);
        if (s != 0) {
            // TODO: Read byte
            //mSamples.append((static_cast<double>(s)) / 32768.0 );
        } else {
            // TODO: Read byte
            //mSamples.append(0);
        }
    }
    qDebug() << "Audio notify size: " << mInputBuffer.size();

    mInputBuffer.buffer().clear();
    mInputBuffer.seek(0);

}

/**
 * @brief MainWindow::stateChangeAudioIn
 * @param s
 */
void MainWindow::stateChangeAudioIn(QAudio::State s) {
    qDebug() << "State change: " << s;
}

/**
 * @brief MainWindow::on_PB_stop_clicked
 */
void MainWindow::on_PB_stop_clicked() {
    // Stop
    mAudioIn->stop();
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
}

/**
 * @brief MainWindow::RefreshInputAudioDevices
 */
void MainWindow::RefreshInputAudioDevices(void) {
    QList<QAudioDeviceInfo> inputDevices = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);

    ui->CB_InputAudioDevice->clear();
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
    bool habhub_en = application_setting.value("Feed/HabHub","false").toBool();
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
    application_setting.setValue("Feed/HabHub",ui->CX_habhubEnable->isChecked());
}
