#include <QCanBus>
#include "newconnectiondialog.h"
#include "ui_newconnectiondialog.h"

NewConnectionDialog::NewConnectionDialog(QVector<QString>* gvretips, QVector<QString>* kayakhosts, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NewConnectionDialog),
    remoteDeviceIPGVRET(gvretips),
    remoteBusKayak(kayakhosts)
{
    ui->setupUi(this);
    if (isSerialBusAvailable())
    {
        ui->rbSocketCAN->setEnabled(true);
        ui->rbSLCAN->setEnabled(true);
    }
    else
    {
        ui->rbSocketCAN->setEnabled(false);
        ui->rbSLCAN->setEnabled(false);
        QString errorString;
        const QList<QCanBusDeviceInfo> devices = QCanBus::instance()->availableDevices(QStringLiteral("socketcan"), &errorString);
        if (!errorString.isEmpty()) ui->rbSocketCAN->setToolTip(errorString);
    }


    connect(ui->rbGVRET, &QAbstractButton::clicked, this, &NewConnectionDialog::handleConnTypeChanged);
    connect(ui->rbSocketCAN, &QAbstractButton::clicked, this, &NewConnectionDialog::handleConnTypeChanged);
    connect(ui->rbRemote, &QAbstractButton::clicked, this, &NewConnectionDialog::handleConnTypeChanged);
    connect(ui->rbKayak, &QAbstractButton::clicked, this, &NewConnectionDialog::handleConnTypeChanged);
    connect(ui->rbMQTT, &QAbstractButton::clicked, this, &NewConnectionDialog::handleConnTypeChanged);
    connect(ui->rbSLCAN, &QAbstractButton::clicked, this, &NewConnectionDialog::handleConnTypeChanged);

    connect(ui->cbDeviceType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &NewConnectionDialog::handleDeviceTypeChanged);
    connect(ui->btnOK, &QPushButton::clicked, this, &NewConnectionDialog::handleCreateButton);

    ui->lblDeviceType->setHidden(true);
    ui->cbDeviceType->setHidden(true);
    selectSerial();

    qDebug() << "Was passed " << remoteDeviceIPGVRET->count() << " remote GVRET IPs";
    qDebug() << "Was passed " << remoteBusKayak->count() << " remote Kayak Busses";
}

NewConnectionDialog::~NewConnectionDialog()
{
    delete ui;
}

void NewConnectionDialog::handleCreateButton()
{
    accept();
}

void NewConnectionDialog::handleConnTypeChanged()
{
    if (ui->rbGVRET->isChecked()) selectSerial();
    if (ui->rbSocketCAN->isChecked()) selectSocketCan();
    if (ui->rbRemote->isChecked()) selectRemote();
    if (ui->rbKayak->isChecked()) selectKayak();
    if (ui->rbMQTT->isChecked()) selectMQTT();
    if (ui->rbSLCAN->isChecked()) selectSLCAN();
}

void NewConnectionDialog::handleDeviceTypeChanged()
{

    ui->cbPort->clear();
    canDevices = QCanBus::instance()->availableDevices(ui->cbDeviceType->currentText());

    for (int i = 0; i < canDevices.count(); i++)
        ui->cbPort->addItem(canDevices[i].name());
}

void NewConnectionDialog::selectSerial()
{
    ui->lPort->setText("Serial Port:");

    ui->lblDeviceType->setHidden(true);
    ui->cbDeviceType->setHidden(true);

    ui->cbPort->clear();
    ports = QSerialPortInfo::availablePorts();

    for (int i = 0; i < ports.count(); i++)
        ui->cbPort->addItem(ports[i].portName());
}

void NewConnectionDialog::selectSocketCan()
{
    ui->lPort->setText("Port:");
    ui->lblDeviceType->setHidden(false);
    ui->cbDeviceType->setHidden(false);

    ui->cbDeviceType->clear();
    QStringList plugins;
    plugins = QCanBus::instance()->plugins();
    for (int i = 0; i < plugins.count(); i++)
        ui->cbDeviceType->addItem(plugins[i]);

}

void NewConnectionDialog::selectRemote()
{
    ui->lPort->setText("IP Address:");
    ui->lblDeviceType->setHidden(true);
    ui->cbDeviceType->setHidden(true);
    ui->cbPort->clear();
    foreach(QString pName, *remoteDeviceIPGVRET)
    {
        ui->cbPort->addItem(pName);
    }
}

void NewConnectionDialog::selectKayak()
{
    ui->lPort->setText("Available Bus(ses):");
    ui->lblDeviceType->setHidden(true);
    ui->cbDeviceType->setHidden(true);
    ui->cbPort->clear();
    foreach(QString pName, *remoteBusKayak)
    {
        ui->cbPort->addItem(pName);
    }
}

void NewConnectionDialog::selectMQTT()
{
    ui->lPort->setText("Topic Name:");
    ui->lblDeviceType->setHidden(true);
    ui->cbDeviceType->setHidden(true);
    ui->cbPort->clear();
}

void NewConnectionDialog::selectSLCAN()
{
    // same as serial
    selectSerial();
}

void NewConnectionDialog::setPortName(CANCon::type pType, QString pPortName, QString pDriver)
{

    switch(pType)
    {
        case CANCon::GVRET_SERIAL:
            ui->rbGVRET->setChecked(true);
            break;
        case CANCon::SERIALBUS:
            ui->rbSocketCAN->setChecked(true);
            break;
        case CANCon::REMOTE:
            ui->rbRemote->setChecked(true);
            break;
        case CANCon::KAYAK:
            ui->rbKayak->setChecked(true);
            break;
        case CANCon::MQTT:
            ui->rbMQTT->setChecked(true);
            break;
        case CANCon::SLCAN:
            ui->rbSLCAN->setChecked(true);
        default: {}
    }

    /* refresh names whenever needed */
    //handleConnTypeChanged();

    switch(pType)
    {
        case CANCon::GVRET_SERIAL:
        case CANCon::SLCAN:
        {
            int idx = ui->cbPort->findText(pPortName);
            if( idx<0 ) idx=0;
            ui->cbPort->setCurrentIndex(idx);
            break;
        }
        case CANCon::SERIALBUS:
        {
            int idx = ui->cbDeviceType->findText(pDriver);
            if (idx < 0) idx = 0;
            ui->cbDeviceType->setCurrentIndex(idx);
            idx = ui->cbPort->findText(pPortName);
            if( idx < 0 ) idx = 0;
            ui->cbPort->setCurrentIndex(idx);
            break;
        }
        case CANCon::REMOTE:
        {
            int idx = ui->cbPort->findText(pPortName);
            if (idx > -1) ui->cbPort->setCurrentIndex(idx);
            else ui->cbPort->addItem(pPortName);
            break;
        }
        case CANCon::KAYAK:
        {
            int idx = ui->cbPort->findText(pPortName);
            if (idx > -1) ui->cbPort->setCurrentIndex(idx);
            else ui->cbPort->addItem(pPortName);
            break;
        }
        case CANCon::MQTT:
            ui->cbPort->setCurrentText(pPortName);
            break;
        default: {}
    }
}

QString NewConnectionDialog::getPortName()
{
    switch( getConnectionType() ) {
    case CANCon::GVRET_SERIAL:
    case CANCon::SERIALBUS:
    case CANCon::REMOTE:
    case CANCon::MQTT:
    case CANCon::SLCAN:
    case CANCon::KAYAK:
        return ui->cbPort->currentText();
    default:
        qDebug() << "getPortName: can't get port";
    }

    return "";
}

QString NewConnectionDialog::getDriverName()
{
    if (getConnectionType() == CANCon::SERIALBUS)
    {
        return ui->cbDeviceType->currentText();
    }
    return "N/A";
}

CANCon::type NewConnectionDialog::getConnectionType()
{
    if (ui->rbGVRET->isChecked()) return CANCon::GVRET_SERIAL;
    if (ui->rbSocketCAN->isChecked()) return CANCon::SERIALBUS;
    if (ui->rbRemote->isChecked()) return CANCon::REMOTE;
    if (ui->rbKayak->isChecked()) return CANCon::KAYAK;
    if (ui->rbMQTT->isChecked()) return CANCon::MQTT;
    if (ui->rbSLCAN->isChecked()) return CANCon::SLCAN;
    qDebug() << "getConnectionType: error";

    return CANCon::NONE;
}

bool NewConnectionDialog::isSerialBusAvailable()
{
    if (QCanBus::instance()->plugins().count() > 0) return true;
    return false;
}
