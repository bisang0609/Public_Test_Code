#ifndef UDPSERVER_H
#define UDPSERVER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QFile>

#include "common.h"

#define CONNECT_TIMEOUT 1000            // 1000ms
#define REQEST_PERIOD   500            // 500ms

class UDPService : public QObject
{
  Q_OBJECT

  public:
      explicit UDPService();
      ~UDPService() override;

      qint64 sendPacket(unsigned char _cmd, int _address, int _parameter);
      qint64 sendPacket(int _address, int _parameter);
      qint64 sendPacket(int _address);

      //==================== EEPROM communication =================================================================//
      qint64 sendPacket(unsigned char _cmd, void *pBuff, int Len);
      qint64 sendEEPROMReqPacket(void);
      qint64 sendEEPROMWritePacket(char *pBuff);
      //==================== EEPROM communication =================================================================//
      void sendPositionPacket(tPosition* _position, uint8_t _nr);

      void sendFile(QFile &_file);

      void setReady();
      void setStandby();
      void setShotmode(int _value);
      void setTargetWait(int _value);
      void setTargetTemp(int _value);
      void setTargetOk(int _value);

      uint8_t  isConnected(void) { return connected; }

  protected:

  private:
      QUdpSocket *socket;
      uint8_t   connected;

      QTimer* requestStateTimer;
      QTimer* connectTimer;

      void _sendFilePayload(void* _data, uint64_t _length, uint64_t _fileSize);

  signals:
      void readyPacket(uint8_t* _buffer, uint16_t _size);
      void reqeustState();
      void handpieceError();

  public slots:
      void readPendingDatagrams();
      void doRequestState();
      void onConnectError();

};

#endif // UDPSERVER_H
