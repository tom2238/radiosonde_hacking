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

    ui->PB_stop->setEnabled(false);
    ui->PB_start->setEnabled(true);
}

/**
 * @brief MainWindow::~MainWindow
 */
MainWindow::~MainWindow()
{
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
    ui->PB_stop->setEnabled(true);
    ui->PB_start->setEnabled(false);
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
    ui->PB_stop->setEnabled(false);
    ui->PB_start->setEnabled(true);
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
