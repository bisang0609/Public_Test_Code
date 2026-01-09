#ifndef SERIALSERVICE_H
#define SERIALSERVICE_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>

class SerialService : public QObject
{
    Q_OBJECT

public:
    SerialService(QObject *parent =  nullptr); // contructor
    ~SerialService();                          // distructor

    virtual void parsingPacket(QByteArray* _packet) = 0;

signals:
    void SIG_serial_error(bool state);

private:
    QSerialPort *qserial;

    QTimer *connect_error_timer;

    QString _getConfigedPort(void);

    void initPort();
    void _initTimer();
    bool _initPort(QSerialPort* _sPort, const QString &_portName);
    void _disconnectPort();
    void _send(QSerialPort* _sPort, const char *data, int maxSize);

private slots:
    void SLT_on_connection_error();
    void SLT_received();

public:
    void send(const char *data, int maxSize);
};

#endif // SERIALSERVICE_H
