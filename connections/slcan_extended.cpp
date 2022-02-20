#include "slcan_extended.h"

#include "canconmanager.h"

#include <QCanBus>
#include <QCanBusFrame>
#include <QDateTime>
#include <QDebug>
#include <QString>
#include <QStringBuilder>
#include <QSerialPortInfo>
#include <QCoreApplication>
#include <QEventLoop>

/***********************************/
/****    CRC8 for binary mode   ****/ 
/***********************************/

/** custom crc8 stuff
 */
const unsigned char crc8_table[256] = {
        0x00, 0x2f, 0x5e, 0x71,
        0xbc, 0x93, 0xe2, 0xcd,
        0x57, 0x78, 0x09, 0x26,
        0xeb, 0xc4, 0xb5, 0x9a,
        0xae, 0x81, 0xf0, 0xdf,
        0x12, 0x3d, 0x4c, 0x63,
        0xf9, 0xd6, 0xa7, 0x88,
        0x45, 0x6a, 0x1b, 0x34,
        0x73, 0x5c, 0x2d, 0x02,
        0xcf, 0xe0, 0x91, 0xbe,
        0x24, 0x0b, 0x7a, 0x55,
        0x98, 0xb7, 0xc6, 0xe9,
        0xdd, 0xf2, 0x83, 0xac,
        0x61, 0x4e, 0x3f, 0x10,
        0x8a, 0xa5, 0xd4, 0xfb,
        0x36, 0x19, 0x68, 0x47,
        0xe6, 0xc9, 0xb8, 0x97,
        0x5a, 0x75, 0x04, 0x2b,
        0xb1, 0x9e, 0xef, 0xc0,
        0x0d, 0x22, 0x53, 0x7c,
        0x48, 0x67, 0x16, 0x39,
        0xf4, 0xdb, 0xaa, 0x85,
        0x1f, 0x30, 0x41, 0x6e,
        0xa3, 0x8c, 0xfd, 0xd2,
        0x95, 0xba, 0xcb, 0xe4,
        0x29, 0x06, 0x77, 0x58,
        0xc2, 0xed, 0x9c, 0xb3,
        0x7e, 0x51, 0x20, 0x0f,
        0x3b, 0x14, 0x65, 0x4a,
        0x87, 0xa8, 0xd9, 0xf6,
        0x6c, 0x43, 0x32, 0x1d,
        0xd0, 0xff, 0x8e, 0xa1,
        0xe3, 0xcc, 0xbd, 0x92,
        0x5f, 0x70, 0x01, 0x2e,
        0xb4, 0x9b, 0xea, 0xc5,
        0x08, 0x27, 0x56, 0x79,
        0x4d, 0x62, 0x13, 0x3c,
        0xf1, 0xde, 0xaf, 0x80,
        0x1a, 0x35, 0x44, 0x6b,
        0xa6, 0x89, 0xf8, 0xd7,
        0x90, 0xbf, 0xce, 0xe1,
        0x2c, 0x03, 0x72, 0x5d,
        0xc7, 0xe8, 0x99, 0xb6,
        0x7b, 0x54, 0x25, 0x0a,
        0x3e, 0x11, 0x60, 0x4f,
        0x82, 0xad, 0xdc, 0xf3,
        0x69, 0x46, 0x37, 0x18,
        0xd5, 0xfa, 0x8b, 0xa4,
        0x05, 0x2a, 0x5b, 0x74,
        0xb9, 0x96, 0xe7, 0xc8,
        0x52, 0x7d, 0x0c, 0x23,
        0xee, 0xc1, 0xb0, 0x9f,
        0xab, 0x84, 0xf5, 0xda,
        0x17, 0x38, 0x49, 0x66,
        0xfc, 0xd3, 0xa2, 0x8d,
        0x40, 0x6f, 0x1e, 0x31,
        0x76, 0x59, 0x28, 0x07,
        0xca, 0xe5, 0x94, 0xbb,
        0x21, 0x0e, 0x7f, 0x50,
        0x9d, 0xb2, 0xc3, 0xec,
        0xd8, 0xf7, 0x86, 0xa9,
        0x64, 0x4b, 0x3a, 0x15,
        0x8f, 0xa0, 0xd1, 0xfe,
        0x33, 0x1c, 0x6d, 0x42
    };

uint8_t crc8_with_init(uint8_t init_value, uint8_t *data, uint8_t len)
{
    uint8_t crc = init_value;
    while (len--) crc = crc8_table[crc ^ *data++];
    return crc;
}

uint8_t crc8(uint8_t *data, uint8_t len){
	return crc8_with_init(0xFF, data, len);
}

void delay(int ms) {
    QTime dieTime= QTime::currentTime().addMSecs(ms);//wait a bit
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

bool SlcanExtendedConnection::waitForResponse(int ms) {
    QTime dieTime= QTime::currentTime().addMSecs(ms);//wait a bit
    while (QTime::currentTime() < dieTime) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        if (rxdata.contains('\r') || cmd_success) {
            cmd_success = false;
            return true;
        } else if (rxdata.contains('\a')) {
            cmd_success = false;
            return false;
        }
    }
    return false;
}


/***********************************/
/****    class definition       ****/
/***********************************/

SlcanExtendedConnection::SlcanExtendedConnection(QString portName) :
    CANConnection(portName, "slcan", CANCon::SLCAN, 3, 4000, true),
    mTimer(this) /*NB: set connection as parent of timer to manage it from working thread */
{
    sendDebug("SLCanExtendedConnection");
    serial = nullptr;
    extended = false;
    binary = false;
    processResults = false;
    cmd_success = false;

    int oldBuses = mBusData.length();
    mBusData.resize(mNumBuses);
    if (mNumBuses > oldBuses)
    {
        for (int i = oldBuses; i < mNumBuses; i++)
        {
            mBusData[i].mConfigured = true;
            mBusData[i].mBus = mBusData[0].mBus;
        }
    }

}


SlcanExtendedConnection::~SlcanExtendedConnection()
{
    stop();
}


void SlcanExtendedConnection::sendDebug(const QString debugText)
{
    qDebug() << "SLCAN: " <<  debugText;
    debugOutput(debugText);
}

void SlcanExtendedConnection::piStarted()
{
    // baud rates to attempt
    // Attempt in order of occurance (most common to least common)
    int baudrates[14] = {
        115200, // most commonly used one
        57200, // default on many devices
        500000, // common fast mode
        1000000, // fastest usable for most avr's (e.g. arduino)
        1500000, // possible on 20MHz avr's
        2000000, // fastest capable for avr's (e.g. custom boards)
        3000000, // fastest possible with 24MHz OC'd avr
        4000000, // fastest supported by linux without custom baud settings
        230400, // double rate 115200, uncommon
        250000, // almost never used as 500k is usually possible on these devices as well
        19200, // default for some classic RS232 ports
        38400, // default on some devices, mostly modems though
        9600, // why? Ah, you like using CAN over teletype!
        2400, // why? Ah, you like using CAN over teletype!
    };
    sendDebug("piStarted()");
    /* create device */
    QString errorString;
    sendDebug("Creating device instance");
    sendDebug("Serial connection slcan device.");

    serial = new QSerialPort(QSerialPortInfo(getPort()));
    if(!serial) {
        sendDebug("can't open serial port " + getPort());
        return;
    }
    sendDebug("Attempting to find baud rate.");

    /* connect reading event */
    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serialError(QSerialPort::SerialPortError)));

    processResults = false;
    connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));

    int rate = 0;
    do {
        if (serial == nullptr) return;
        /* configure */
        sendDebug("Baud rate attempt: " % QString::number(baudrates[rate]));
        serial->setBaudRate(baudrates[rate]); // set for 2 megabaud
        serial->setDataBits(serial->Data8);

        serial->setFlowControl(serial->HardwareControl); // USB Device, so hardware flow ctrol
        if (!serial->open(QIODevice::ReadWrite))
        {
            if (serial != nullptr) sendDebug("Error returned during port opening: " + serial->errorString());
        } else {
            serial->clear();
        }
        // clear buffers of device
        QByteArray data;
        data.fill('\r', 32);
        sendToSerial(data);
        // attempt to get serial, should be only numbers
        if (getVersion()) {
            baudrate = baudrates[rate];
            sendDebug("Baud rate found: " % QString::number(baudrate));
            break; 
        }
        // serial can be null if it is in use
        if (nullptr!= serial && serial->isOpen()) serial->close();
        rate++;
        rxdata.clear();
    } while(rate < 14);


    mNumBuses = 1; // fix for now, detect later
    extended = false; // fix for now, detect later

    canbus_settings = (slcan_settings *) calloc(sizeof(slcan_settings), mNumBuses);

    connectDevice();
}


void SlcanExtendedConnection::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void SlcanExtendedConnection::piStop() {
    sendDebug("piStop()");
    mTimer.stop();
    disconnectDevice();
}


bool SlcanExtendedConnection::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    sendDebug("About to get bus " % QString::number(pBusIdx));
    return getBusConfig(pBusIdx, pBus);
}


void SlcanExtendedConnection::piSetBusSettings(int pBusIdx, CANBus bus)
{
    //CANConStatus stats;
    /* sanity checks */

    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    sendDebug("About to update bus " % QString::number(pBusIdx));
    if (bus.getSpeed() != mBusData[pBusIdx].mBus.getSpeed()) {
        sendState(-1, CLOSED);
        sendDebug("Bus speed " % QString::number(bus.getSpeed()));
        sendBaud(pBusIdx, bus.getSpeed());
    }
    
    setBusConfig(pBusIdx, bus);

    if (mNumBuses == 1) {
        if (mBusData[0].mBus.isListenOnly() && mBusData[0].mBus.isActive()) {
            sendState(0, LISTEN);
        } else if (mBusData[0].mBus.isActive()) {
            sendState(0, OPEN);
        }
    } else {
        for (int i = 0; i < mNumBuses; i++) {
            if (mBusData[i].mBus.isListenOnly() && mBusData[i].mBus.isActive()) {
                sendState(i, LISTEN);
            } else if (mBusData[i].mBus.isActive()) {
                sendState(i, OPEN);
            }
        }
    }
}


bool SlcanExtendedConnection::piSendFrame(const CANFrame& frame)
{
    uint32_t id;
    int bus;
    int len;

    switch (frame.frameType()) {
        case QCanBusFrame::FrameType::RemoteRequestFrame:
        case QCanBusFrame::FrameType::DataFrame:
            break;
        default:
            return true;
    }
    qDebug() << "Frametype: " << frame.frameType();
    qDebug() << "ID: " << frame.frameId();
    qDebug() << "Data: " << frame.payload().toHex();


    id = frame.frameId();
    bus = extended ? frame.bus : -1;
    len = frame.payload().length();

    return sendCANFrame(bus, id,
            frame.frameType() == QCanBusFrame::FrameType::RemoteRequestFrame ? true : false,
            frame.hasExtendedFrameFormat(),
            (const uint8_t *) frame.payload().constData(), len);
}


/***********************************/
/****   private methods         ****/
/***********************************/


/* disconnect device */

void SlcanExtendedConnection::serialError(QSerialPort::SerialPortError err)
{
    QString errMessage;
    bool killConnection = false;
    switch (err)
    {
    case QSerialPort::NoError:
        return;
    case QSerialPort::DeviceNotFoundError:
        errMessage = "Device not found error on serial";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::PermissionError:
        errMessage =  "Permission error on serial port";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::OpenError:
        errMessage =  "Open error on serial port";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::ParityError:
        errMessage = "Parity error on serial port";
        break;
    case QSerialPort::FramingError:
        errMessage = "Framing error on serial port";
        break;
    case QSerialPort::BreakConditionError:
        errMessage = "Break error on serial port";
        break;
    case QSerialPort::WriteError:
        errMessage = "Write error on serial port";
        piStop();
        break;
    case QSerialPort::ReadError:
        errMessage = "Read error on serial port";
        piStop();
        break;
    case QSerialPort::ResourceError:
        errMessage = "Serial port seems to have disappeared.";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::UnsupportedOperationError:
        errMessage = "Unsupported operation on serial port";
        killConnection = true;
        break;
    case QSerialPort::UnknownError:
        errMessage = "Beats me what happened to the serial port.";
        killConnection = true;
        piStop();
        break;
    case QSerialPort::TimeoutError:
        errMessage = "Timeout error on serial port";
        killConnection = true;
        break;
    case QSerialPort::NotOpenError:
        errMessage = "The serial port isn't open";
        killConnection = true;
        piStop();
        break;
    }
    /*
    if (serial)
    {
        serial->clearError();
        serial->flush();
        serial->close();
    }*/
    if (errMessage.length() > 1)
    {
        sendDebug(errMessage);
    }
    if (killConnection)
    {
        sendDebug("Shooting the serial object in the head. It deserves it.");
        disconnectDevice();
    }
}

void SlcanExtendedConnection::connectDevice() {
    QByteArray buffer;
    bool failed = false;

    // send close no matter what
    sendState(-1, CLOSED);

    // detect if extended will fail on incompatible devices with [BELL] (0x07 aka \a)
    buffer.append(SLCAN_CMD_EXTENDED_BINARY);
    buffer.append('0');
    buffer.append(SLCAN_CMD_OK);


    rxdata.clear();
    sendToSerial(buffer);
    delay(50);

    // all responses should be 1 byte (fail or success) everything else is considered resigual garbadge
    if (rxdata.length() > 1) {
        sendDebug("Second attempt");
        rxdata.clear();
        sendToSerial(buffer);
        delay(50);
    }

    // default 
    extended = false;
    binary = false;

    if (rxdata.length() != 1) {
        sendDebug("Invalid data detected, maybe try a different baud mode");
        failed = true;
    } else {
        if (rxdata[0] == SLCAN_CMD_OK) {
            extended = true;
            binary = true;
            sendDebug("Extended protocol supported");
        } else {
            sendDebug("Using default protocol");
        }
    }

    rxdata.clear(); // remove leftover data

    if (extended) {
        // detect busses
        int busmax;
        int oldBuses = mNumBuses;
        for (busmax = 1; busmax < 10; busmax++) {
            buffer.clear();
            buffer.append(SLCAN_CMD_EXTENDED_BUS);
            buffer.append(0x30 + busmax);
            buffer.append(SLCAN_CMD_CLOSE); // must send some command
            buffer.append(SLCAN_CMD_OK);

            sendToSerial(buffer);

            if (!waitForResponse(10)) {
                break;
            }
            rxdata.clear();
        }
        sendDebug("Found " % QString::number(busmax) % " busses");
        mNumBuses = busmax;

        mBusData.resize(mNumBuses);
        if (mNumBuses > oldBuses)
        {
            for (int i = oldBuses; i < mNumBuses; i++)
            {
                mBusData[i].mConfigured = true;
                mBusData[i].mBus = mBusData[0].mBus;
            }
        }
    }

    for (int i = 0; i < mNumBuses && !failed; i++) {
        sendDebug("Setting bus " % QString::number(i));
        sendBaud(i, mBusData[i].mBus.getSpeed());
        mBusData[i].mConfigured = true;
    }


    if (!sendState(-1, OPEN)) {
        failed = true;
    }


    if (!failed) {
        sendDebug("Connected");
        setStatus(CANCon::CONNECTED);
    } else {
        sendDebug("Not Connected");
        setStatus(CANCon::NOT_CONNECTED);
    }

    // enabled!

    // set binary mode
    setBinary(binary);

    // enable processing
    processResults = true;

    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);
}

void SlcanExtendedConnection::disconnectDevice() {

    disconnect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));

    if (serial != nullptr) {
        if (serial->isOpen()) {
            serial->close();
        }
        serial->disconnect(); //disconnect all signals
        delete serial;
        serial = nullptr;
    }
    setStatus(CANCon::NOT_CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);
}


void SlcanExtendedConnection::sendBaud(int bus, int rate) {
    QByteArray buf;
    rxdata.clear();

    buf.append(SLCAN_CMD_EXTENDED_BUS);
    buf.append(0x30 + bus);
    buf.append(SLCAN_CMD_BAUD);
    char setting = '0';
    switch (rate) {
        case 10000:
            setting = '0';
            break;
        case 20000:
            setting = '1';
            break;
        case 50000:
            setting = '2';
            break;
        case 100000:
            setting = '3';
            break;
        case 125000:
        default:
            setting = '4';
            break;
        case 250000:
            setting = '5';
            break;
        case 500000:
            setting = '6';
            break;
        case 800000:
            setting = '7';
            break;
        case 1000000:
            setting = '8';
            break;
    }
    buf.append(setting);
    buf.append(SLCAN_CMD_OK);
    sendToSerial(buf);
    // check result just for us
    if (!waitForResponse(50)) {
        sendDebug("Failed to set can bus bitrate");
    }
    rxdata.clear();
}

void SlcanExtendedConnection::setBinary(bool mode) {
    if (!extended) return;

    rxdata.clear();
    QByteArray buf;
    buf.append(SLCAN_CMD_EXTENDED_BINARY);
    buf.append(0x30 + (mode ? 1 : 0)); // 1 for enable, 0 for disable
    buf.append(SLCAN_CMD_OK);
    sendToSerial(buf);

    
    if (!waitForResponse(10)) {
        sendDebug("Failed to set binary mode");
    }
    rxdata.clear();
}

void SlcanExtendedConnection::connectionTimeout()
{
}

void SlcanExtendedConnection::readSerialData()
{
    QByteArray data;
    unsigned char c;
    QString debugBuild;

    if (serial == nullptr) {
        sendDebug("Attempting to read uninitialized serial port");
        return;
    }
    data = serial->readAll();
    debugBuild = debugBuild % "[" % QString::number(data.length()) % "]: ";
    for (int i = 0; i < data.length(); i++)
    {
        c = data.at(i);
        rxdata.append(c);
        debugBuild = debugBuild % QString::number(c, 16) % " ";
    }
    sendDebug("Result: " + rxdata);
    if (processResults) processData();
    sendDebug(debugBuild);
}

void SlcanExtendedConnection::processData() {
    // process recieved data 

    QByteArray payload;
    CANFrame frame, *frame_p;

    if (rxdata.length() < 1) return;

    // 0xA5 should never be in any SLCAN data packet which is why it can be detected easilly
    if (binary && rxdata.contains(0xA5) && rxdata.length() >= (int) sizeof(slcan_binary)) {  // explicit conversion due to c++ being c??
        // strip garbadge
        while(rxdata.length() > 0 && rxdata.contains(0xA5)) {
            int index = rxdata.indexOf(0xA5);
            rxdata.remove(0, index);
            qDebug() << "Looking at: " << rxdata;

            slcan_binary data;
            const char *raw = rxdata.constData(); // should work
            memcpy(&data, raw, sizeof(slcan_binary));
            uint8_t crc = crc8((uint8_t *) &data, sizeof(slcan_binary) - 1);
            if (crc == data.crc) {
                // frame, process
                frame_p = getQueue().get();
                if(!frame_p) {
                    // no frame, dump it nowhere
                    qDebug() << "can't get a frame, ERROR";
                    return;
                }
                // convert ID into 'id'
                int id = gID(data.id);
                qDebug() << "RECV: " << id << " | " << data.len << " | " << QByteArray(reinterpret_cast<const char *>(data.data), data.len);
                frame.setFrameType(gRTR(id) ? QCanBusFrame::FrameType::RemoteRequestFrame : QCanBusFrame::FrameType::DataFrame);
                frame.setPayload(QByteArray(reinterpret_cast<const char *>(data.data), data.len));
                frame.setFrameId(id);
                frame.setExtendedFrameFormat(gID(data.id) > 0x7FF);
                frame.bus = gBUS(data.id);
                // slcan time is inaccurate anyway and only lasts 1 second or is otherwise something else if
                // you use non standardised slcan variants. Use system time instead.
                frame.setTimeStamp(QCanBusFrame::TimeStamp(0, (QDateTime::currentMSecsSinceEpoch() * 1000l) - CANConManager::getInstance()->getTimeBasis()));
                checkTargettedFrame(frame);

                *frame_p = frame;
                getQueue().queue();
            } else {
                qDebug() << "Frame CRC error: " << crc << " != " << data.crc;
            }
            rxdata.remove(0, sizeof(slcan_binary));
        }
    // Normal mode, only read if containing \r
    } else if (rxdata.contains(SLCAN_CMD_OK)) {
        while(rxdata.length() > 0 && rxdata.contains(SLCAN_CMD_OK)) {
            int bus = 0;
            if (extended) {
                if (rxdata[0] == SLCAN_CMD_EXTENDED_BUS) {
                    bus = rxdata[1] - 0x30;
                    qDebug() << "Bus " << bus;
                    rxdata.remove(0, 2);
                }
            }

            bool ext = false, rtr = false;
            int id, len;
            switch (rxdata[0]) {
                case SLCAN_CMD_TRANSMIT_RTR_EXT:
                    rtr = true;
                    // fall through
                case SLCAN_CMD_TRANSMIT_EXT:
                    ext = true;
                    // fall through
                case SLCAN_CMD_TRANSMIT_RTR:
                    if (!ext) rtr = true;
                    // fall through
                case SLCAN_CMD_TRANSMIT:
                    //
                    frame_p = getQueue().get();
                    if(!frame_p) {
                        // no frame, dump it nowhere
                        qDebug() << "can't get a frame, ERROR";
                        break;
                    }
                    id = rxdata.mid(1, ext ? 8 : 3).toInt(nullptr, 16);
                    len = rxdata.mid(ext ? 1 + 8 : 1 + 3, 1).toInt(nullptr, 16);
                    payload = rxdata.mid(ext ? 1 + 8 + 1 : 1 + 3 + 1, len * 2);
                    qDebug() << "RECV: " << id << " | " << len << " | " << payload;
                    frame.setFrameType(rtr ? QCanBusFrame::FrameType::RemoteRequestFrame : QCanBusFrame::FrameType::DataFrame);
                    frame.setPayload(QByteArray::fromHex(payload));
                    frame.setFrameId(id);
                    frame.setExtendedFrameFormat(ext);
                    frame.bus = bus;
                    // slcan time is inaccurate anyway and only lasts 1 second or is otherwise something else if
                    // you use non standardised slcan variants. Use system time instead.
                    qDebug() << QDateTime::currentMSecsSinceEpoch();
                    qDebug() << CANConManager::getInstance()->getTimeBasis();
                    frame.setTimeStamp(QCanBusFrame::TimeStamp(0, (QDateTime::currentMSecsSinceEpoch() * 1000l) - CANConManager::getInstance()->getTimeBasis()));
                    checkTargettedFrame(frame);
                    *frame_p = frame;
                    getQueue().queue();
                    
                    break;
                default:
                    break;
            }
            if (rxdata.contains(SLCAN_CMD_OK)) {
                rxdata.remove(0, rxdata.indexOf(SLCAN_CMD_OK) + 1);
                cmd_success = true;
            }
        }
        if (!rxdata.contains(SLCAN_CMD_OK)) {
            if (rxdata.length() > (int) SLCAN_MAX_EXT_SEND_LEN) rxdata.clear(); // mop up data
            return;
        }//  && !rxdata.contains(SLCAN_CMD_ERROR)
    // error recieved
    } else if (rxdata.contains(SLCAN_CMD_ERROR)) {
        rxdata.remove(0, rxdata.indexOf(SLCAN_CMD_ERROR));
        cmd_success = false;
    // if not wait for buffer to fill up (SLCAN has no way to detect a partial message without \r or \a
    } else if (rxdata.length() > (int) SLCAN_MAX_EXT_SEND_LEN) {
        rxdata.clear();
        cmd_success = false;
    } // else is not enough data received
}

void SlcanExtendedConnection::sendToSerial(const QByteArray &bytes)
{
    if (serial == nullptr)
    {
        sendDebug("Attempt to write to serial port when it has not been initialized!");
        return;
    }

    if (!serial->isOpen())
    {
        sendDebug("Attempt to write to serial port when it is not open!");
        return;
    }


    QString buildDebug;
    buildDebug = "Write to serial " % QString::number(bytes.length()) % "-> ";
    foreach (int byt, bytes) {
        byt = (unsigned char)byt;
        buildDebug = buildDebug % QString::number(byt, 16) % " ";
    }
    sendDebug(buildDebug);

    serial->write(bytes);
}

bool SlcanExtendedConnection::getVersion() {
    // version is always a number, if your configuration has something else
    // this will fail and is out of the original LAWICEL spec. 
    // This is used for detecting the baud rate of the COM connection
    QByteArray data;
    data.append(SLCAN_CMD_VERSION);
    data.append(SLCAN_CMD_OK);

    rxdata.clear();
    sendToSerial(data);

    waitForResponse(100);

    if (rxdata.startsWith('Q')) rxdata.remove(0, rxdata.indexOf('\r')+1);
    if (rxdata.length() != 6) return false;
    if (rxdata[0] != SLCAN_CMD_VERSION && rxdata[5] != SLCAN_CMD_OK) return false;
    for (int i = 1; i < 5; i++) if (rxdata[i] > '9' || rxdata[i] < '0') return false;

    return true;
}

bool SlcanExtendedConnection::sendState(int bus, enum SLCAN_STATE state) {
    QByteArray data;
    if (bus > 0 && bus < mNumBuses && bus < 10) {
        data.append(SLCAN_CMD_EXTENDED_BUS);
        data.append('0' + bus);
    }

    switch (state) {
        case OPEN:
            data.append(SLCAN_CMD_OPEN);
            break;
        case CLOSED:
            data.append(SLCAN_CMD_CLOSE);
            break;
        case LISTEN:
            data.append(SLCAN_CMD_LISTEN);
            break;
        default:
            return false;
    }

    data.append(SLCAN_CMD_OK);

    rxdata.clear();
    sendToSerial(data);

    // data should be there quite quickly

    if (!waitForResponse(10)) {
        if (bus > 0) {
            mBusData[bus].mBus.setActive(false);
        } else {
            for (int i = 0; i < mNumBuses; i++) mBusData[i].mBus.setActive(false);
        }
        return false;
    }

    switch (state) {
        case CLOSED:
            if (bus > 0) {
                mBusData[bus].mBus.setActive(false);
            } else {
                for (int i = 0; i < mNumBuses; i++) mBusData[i].mBus.setActive(false);
            }
            break;
        case OPEN:
            if (bus > 0) {
                mBusData[bus].mBus.setActive(true);
            } else {
                for (int i = 0; i < mNumBuses; i++) mBusData[i].mBus.setActive(true);
            }
            break;
        case LISTEN:
            if (bus > 0) {
                mBusData[bus].mBus.setActive(true);
                mBusData[bus].mBus.setListenOnly(true);
            } else {
                for (int i = 0; i < mNumBuses; i++) {
                    mBusData[i].mBus.setActive(true);
                    mBusData[i].mBus.setListenOnly(true);
                }
            }
    }
    return true;
}

bool SlcanExtendedConnection::sendCANFrame(int bus, int id, bool rtr, bool ext, const uint8_t *data, const uint8_t len) {
    QByteArray msg;
    msg.clear();

    if (!binary) {
        if (bus >= 0) {
            msg.append(SLCAN_CMD_EXTENDED_BUS);
            msg.append(0x30 + bus);
        }
        uint8_t cmd = rtr ? SLCAN_CMD_TRANSMIT_RTR : SLCAN_CMD_TRANSMIT;
        if (ext) cmd -= 0x20; // lower case is offset by 0x20
        msg.append(cmd);
        QByteArray idstring = QStringLiteral("%1").arg(id, id > 0x7ff ? 8 : 3, 16, QLatin1Char('0')).toUpper().toUtf8();
        msg.append(idstring.mid(0, idstring.length()));
        msg.append(0x30 + len);
        for (int i = 0; i < len; i++) {
            QByteArray bt = QStringLiteral("%1").arg(data[i], 2, 16, QLatin1Char('0')).toUpper().toUtf8();
            msg.append(bt.mid(0, bt.length()));
        }
        msg.append(SLCAN_CMD_OK);
    } else {
        slcan_binary frm;
        memset(&frm, 0x00, sizeof(slcan_binary));
        frm.preamble = 0xA5;
        frm.id = IDFIELD(bus, rtr, id);
        frm.len = len;
        memcpy(frm.data, data, len);
        frm.crc = crc8((uint8_t *)&frm, sizeof(slcan_binary) - 1);
        msg.append((const char *) &frm, sizeof(slcan_binary));
    }

    sendToSerial(msg);

    if (!binary) {
        return waitForSendSignal(10);
    }
    return true; // always assume success
}

bool SlcanExtendedConnection::waitForSendSignal(int ms) {
    // just a signal
    int counter = 0;
    while(counter < ms) {
        if (rxdata.contains('z')) {
            rxdata.remove(rxdata.indexOf('z', 0), 2);
            return true;
        } else if (rxdata.contains('Z')) {
            rxdata.remove(rxdata.indexOf('Z', 0), 2);
            return true;
        }
        delay(1);
        counter++;
    }
    qDebug() << "Timeout";
    return false;
}
