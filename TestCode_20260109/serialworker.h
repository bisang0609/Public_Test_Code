#ifndef SERIALWORKER_H
#define SERIALWORKER_H

#include "serialservice.h"
#include "common.h"


class SerialWorker : public SerialService
{
    Q_OBJECT

public:
     explicit SerialWorker(SerialService *parent = 0);
     ~SerialWorker();

     void sendActionPacket(unsigned char _cmd, int _address, int _parameter);
     void sendActionPacket(int _address, int _parameter);
     void sendActionPacket(int _address);

     void sendStatusPacket(int _temperature, int _status);
     void sendPositionPacket(tPosition* _position, uint8_t _nr);

     //==================== EEPROM communication =================================================================//
     void sendEEPROMPacket(unsigned char _cmd, unsigned char *_sROMData);
     //==================== EEPROM communication =================================================================//
protected:


private:
     void parsingPacket(QByteArray* _data);        // virtual
     bool isHeader(uint8_t _firstHeader, uint8_t _secondHeader);

private slots:

signals:
     void readyPacket(QByteArray _packet);
};

#endif // SERIALWORKER_H
