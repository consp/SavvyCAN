#ifndef SLCAN_EXTENDED_H
#define SLCAN_EXTENDED_H

#include "canconnection.h"
#include "canframemodel.h"

#include <QCanBusDevice>
#include <QSerialPort>
#include <QTimer>

// supported commands
#define SLCAN_CMD_EXTENDED_BINARY   'D'
#define SLCAN_CMD_EXTENDED_BUS      'B'
#define SLCAN_CMD_LISTEN            'L'
#define SLCAN_CMD_OPEN              'O'
#define SLCAN_CMD_CLOSE             'C'
#define SLCAN_CMD_OK                '\r'
#define SLCAN_CMD_ERROR             '\a'
#define SLCAN_CMD_BAUD              'S'
#define SLCAN_CMD_TRANSMIT          't'
#define SLCAN_CMD_TRANSMIT_EXT      'T'
#define SLCAN_CMD_TRANSMIT_RTR      'r'
#define SLCAN_CMD_TRANSMIT_RTR_EXT  'R'
#define SLCAN_CMD_TIMESTAMP         'Z'
#define SLCAN_CMD_VERSION           'V'


#define SLCAN_MAX_SEND_LEN_STRING "B0T001122338001122334455667788EA5F\r"
#define SLCAN_MAX_SEND_LEN (sizeof(SLCAN_MAX_SEND_LEN_STRING) + 1)
#define SLCAN_MAX_EXT_SEND_LEN (SLCAN_MAX_SEND_LEN - 2) 

// binary extension
#define SLCAN_BINARY_MTU 15

#define gBUS(idfield) ((idfield & 0xC0000000) >> 30)
#define gRTR(idfield) ((idfield & 0x20000000) >> 29)
#define gID(idfield) (idfield & 0x1FFFFFFF)

#define IDFIELD(bus, rtr, id)  ((id & 0x1FFFFFFF) | (0x20000000 & (rtr << 29)) | (0xC0000000 & (bus << 30)))

#define SLCAN_BINARY_PREAMBLE 0xA5
struct slcan_binary {
    uint8_t preamble;
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    uint8_t crc;
}__attribute__((packed));

enum SLCAN_STATE {
    CLOSED,
    LISTEN,
    OPEN
};

typedef struct slcan_settings_t {
    int baudrate;
    enum SLCAN_STATE state;
} slcan_settings;

class SlcanExtendedConnection : public CANConnection
{
    Q_OBJECT

public:
    SlcanExtendedConnection(QString portName);
    virtual ~SlcanExtendedConnection();

protected:

    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CANFrame&);

private slots:

    void connectDevice();
    void disconnectDevice();
    void connectionTimeout();
    void readSerialData();
    void serialError(QSerialPort::SerialPortError err);

protected:
    QSerialPort         *serial;
    QTimer              mTimer;
private:
    void sendDebug(const QString debugText);
    void sendToSerial(const QByteArray &bytes);
    void sendBaud(int bus, int rate);
    void setBinary(bool mode);
    bool getVersion();
    bool sendState(int bus, enum SLCAN_STATE state);
    bool sendCANFrame(int bus, int id, bool rtr, bool ext, const uint8_t *data, const uint8_t len); 
    bool waitForSendSignal(int ms);
    void processData();
    bool waitForResponse(int ms);

    bool extended;
    bool binary;
    int baudrate;
    slcan_settings *canbus_settings;
    QByteArray rxdata;
    int sendCounter;
    bool processResults;
};


#endif // SLCAN_EXTENDED_H
