#ifndef SLCAN_EXTENDED_H
#define SLCAN_EXTENDED_H

/**
 * SLCAN_EXTENDED.H
 *
 * @Author consp (github.com/consp)
 * @brief This connection method should support the basic slcan/lawincel protocol used by some devices. It is by no means complete support.
 *        Features: - baud rate detection
 *                  - Extended binary protocol with data intengrity check (not in SLCAN spec.)
 *        Requirements: - Timestamps should be disabled, they are unreliable at best, we always use the system clock no matter what the general setting is.
 *                      - Baud rate must be set to supported values, support is limited to and attempted in this order: 115200, 57200, 500000, 1000000, 1500000, 2000000, 3000000, 4000000, 230400, 250000, 19200, 38400, 9600, 2400
 *
 *
 **/

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

// primitives to fetch data
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
    /**
     * @brief set baud rate and send to slcan device
     * @param bus bus to send it to
     * @param rate baud rate in integer specified by slcan
     */
    void sendBaud(int bus, int rate);
    /**
     * @brief set binary mode if support is detected
     * @param mode true for binary, false for slcan
     */
    void setBinary(bool mode);
    /**
     * @brief get the version from the device, this is used to detect if it is alive and baud rate is correct
     * @return true if version detected, false otherwise
     */
    bool getVersion();
    /**
     * @brief set bus to closed/listen/open
     * @param bus   bus to set
     * @param state state to set the bus to
     * @return  true if success, false otherwise
     */
    bool sendState(int bus, enum SLCAN_STATE state);
    /**
     * @brief send a can frame to the slcan device
     * @param bus       bus to send to
     * @param id        CANid to send
     * @param rtr       true to send rtr frame
     * @param ext       true to send extended frame
     * @param *data     pointer to binary data
     * @param len       length of *data
     * @return true if success, false otherwise.
     */
    bool sendCANFrame(int bus, int id, bool rtr, bool ext, const uint8_t *data, const uint8_t len); 
    /**
     * @brief wait for a succes/failure response after sending for [n] ms
     * @param ms    milliseconds to wait for response
     * @return true if success false otherwise
     */
    bool waitForSendSignal(int ms);
    /**
     * @brief process received data
     */
    void processData();
    /**
     * @brief wait for a response for [n] ms
     * @param ms Milliseconds to wait
     * @return true if success false otherwise
     */
    bool waitForResponse(int ms);

    bool extended;
    bool binary;
    int baudrate;
    slcan_settings *canbus_settings;
    QByteArray rxdata;
    int sendCounter;
    bool processResults;
    bool cmd_success;
};


#endif // SLCAN_EXTENDED_H
