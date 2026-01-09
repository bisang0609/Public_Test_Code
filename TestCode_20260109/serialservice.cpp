/***
 * Serial(RS-232) 통신을 담당
 */

#include "serialservice.h"

#include <QDebug>
#include <QThread>


SerialService::SerialService(QObject *parent) :
    QObject(parent),
    qserial(new QSerialPort)
{
#ifndef DISABLE_USART
    initPort();
    _initTimer();
#endif
}


SerialService::~SerialService()
{
    connect_error_timer->stop();
    this->_disconnectPort();
    delete qserial;
    qserial = nullptr;
}


/***
 * VSLS Serial Port를 구함
 * Manufacturer: FTDI
 */
QString SerialService::_getConfigedPort(void)
{
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        if (serialPortInfo.manufacturer() == "FTDI" &&
            serialPortInfo.vendorIdentifier() == 1027 &&
            serialPortInfo.productIdentifier() == 24597)
                return serialPortInfo.portName();
    }

    return "";
}


void SerialService::initPort()
{
#ifdef DISABLE_USART
    QString portName = _getConfigedPort();

    connect(qserial, SIGNAL(readyRead()), this, SLOT(SLT_received()), Qt::DirectConnection);

    _initPort(this->qserial, portName);
#endif
}


bool SerialService::_initPort(QSerialPort* _sPort, const QString &_portName)
{
    if (_sPort->isOpen()) {
        _sPort->clear();
        _sPort->close();
    }

    _sPort->setPortName(_portName);
    _sPort->setBaudRate(QSerialPort::Baud115200); //9600 for test,  115200 for a product.
    _sPort->setDataBits(QSerialPort::Data8);
    _sPort->setParity(QSerialPort::NoParity);
    _sPort->setStopBits(QSerialPort::OneStop);
    _sPort->setFlowControl(QSerialPort::NoFlowControl);
//    _sPort->setReadBufferSize(4096);

    qInfo() << "Serial Port Name" + _portName;
    if(!_sPort->open(QIODevice::ReadWrite))
    {
        qInfo() << _portName + " Serial Port Error";
        return false;
    }

    return true;
}



#define TIMEOUT         1500        // ms

void SerialService::_initTimer(){

    connect_error_timer = new QTimer();

    connect(connect_error_timer, SIGNAL(timeout()), this, SLOT(SLT_on_connection_error()));

    connect_error_timer->start(TIMEOUT);
}


void SerialService::_disconnectPort()
{
    qserial->close();
    qserial = nullptr;
}

void SerialService::SLT_received()
{
    static QByteArray   receivedData;

    if(qserial->bytesAvailable() > 0)
    {
        receivedData = qserial->readAll();
        parsingPacket(&receivedData);
    }

    connect_error_timer->start(TIMEOUT);
}

void SerialService::SLT_on_connection_error()
{
    initPort();

    emit SIG_serial_error(false);
}

void SerialService::send(const char *data, int maxSize)
{
    _send(this->qserial, data, maxSize);
}

void SerialService::_send(QSerialPort* _sPort, const char *data, int maxSize)
{
    if (!_sPort->isOpen()) return;

    qint64 ret =  _sPort->write(data, maxSize);

    _sPort->flush();
    //qserial->waitForBytesWritten(100);
    if(ret != maxSize)
    {
        qInfo() << "Serial Sending Error";
    }
}
