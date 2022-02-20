#ifndef NEWCONNECTIONDIALOG_H
#define NEWCONNECTIONDIALOG_H

#include <QDialog>
#include <QCanBusDeviceInfo>
#include <QSerialPortInfo>
#include <QDebug>
#include <QUdpSocket>
#include "canconnectionmodel.h"
#include "connections/canconnection.h"

namespace Ui {
class NewConnectionDialog;
}

class NewConnectionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit NewConnectionDialog(QVector<QString>* gvretips, QVector<QString>* kayakips, QWidget *parent = nullptr);
    ~NewConnectionDialog();

    CANCon::type getConnectionType();
    QString getPortName();
    QString getDriverName();

public slots:
    void handleConnTypeChanged();
    void handleDeviceTypeChanged();
    void handleCreateButton();

private:
    Ui::NewConnectionDialog *ui;
    QList<QSerialPortInfo> ports;
    QList<QCanBusDeviceInfo> canDevices;
    QVector<QString>* remoteDeviceIPGVRET;
    QVector<QString>* remoteBusKayak;

    void selectSerial();
    void selectKvaser();
    void selectSocketCan();
    void selectRemote();
    void selectKayak();
    void selectMQTT();
    void selectSLCAN();
    bool isSerialBusAvailable();
    void setPortName(CANCon::type pType, QString pPortName, QString pDriver);
};

#endif // NEWCONNECTIONDIALOG_H
