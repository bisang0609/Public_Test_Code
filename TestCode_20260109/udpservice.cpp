#include "udpservice.h"


#include <QFileInfo>

QString PC_addr = "192.168.1.150";
QString HP_addr = "192.168.1.70";

uint16_t port = 7;

UDPService::UDPService()
{
    socket = new QUdpSocket(this);

    /* We need to bind the UDP socket to an address and a port */
    socket->bind(QHostAddress(PC_addr), port);         //ex. Address localhost, port 1234

    connect(socket,SIGNAL(readyRead()),this,SLOT(readPendingDatagrams()), Qt::DirectConnection);

    /* HandPiece로 주기적으로 보내는 타이머 */
//#define REQEST_PERIOD   500            // 500ms
    requestStateTimer = new QTimer();
    connect(requestStateTimer,SIGNAL(timeout()),this,SLOT(doRequestState()));
    requestStateTimer->start(REQEST_PERIOD);

    /* HandPiece의 응답을 주기적으로 체크하는 타이머*/
//#define CONNECT_TIMEOUT 1000            // 1000ms
    connectTimer = new QTimer();
    connect(connectTimer, SIGNAL(timeout()), this, SLOT(onConnectError()));

    /* ========== 20220616 JYH Append ========================================= */
    connectTimer->setSingleShot(true);
    /* ========== 20220616 JYH Append ========================================= */

    connectTimer->start(CONNECT_TIMEOUT*2);     /* 처음은 2초간 */

    connected = false;
}


UDPService::~UDPService()
{
    requestStateTimer->stop();
    connectTimer->stop();

    delete requestStateTimer;
    delete connectTimer;
    delete socket;
}


void UDPService::setReady()
{
    sendPacket(SET_OPMODE, READY);
}

void UDPService::setStandby()
{
    sendPacket(SET_OPMODE, STANDBY);
}

void UDPService::setShotmode(int _value)
{
    sendPacket(SET_SHOT_MODE, _value);
}

void UDPService::setTargetWait(int _value)
{
    sendPacket(SET_TEC_CHECKING, _value);
}

void UDPService::setTargetTemp(int _value)
{
    sendPacket(SET_TARGET_TEMP, _value);
}

void UDPService::setTargetOk(int _value)
{
    sendPacket(SET_TARGET_OK, _value);
}


/***
 * udp receive slot
 */
void UDPService::readPendingDatagrams()
{
    static uint8_t  datagram[IMAGE_SIZE] = {0, };
    static uint16_t sizeOfDatagram = 0;

    /* ========== 20220616 JYH Append ========================================= */
    if(connectTimer->isActive())
        connectTimer->stop();
    /* ========== 20220616 JYH Append ========================================= */

    sizeOfDatagram = socket->pendingDatagramSize();

    socket->readDatagram((char *)datagram, sizeOfDatagram,
                         nullptr, nullptr);// data, maxsize, host addr, host portqbytearray binary unpack

    connectTimer->start(CONNECT_TIMEOUT*2);
    connected = true;

    emit readyPacket(datagram, sizeOfDatagram);
}

qint64 UDPService::sendPacket(unsigned char _cmd, int _address, int _parameter)
{
    tPacket _packet;

    _packet.Header  = HEADER;
    _packet.Header2 = HEADER;
    _packet.Command = _cmd;
    _packet.Address = _address;
    _packet.Parameter = _parameter;
    _packet.Tail    = TAIL;
    _packet.Tail2   = TAIL;

    return (socket->writeDatagram(QByteArray(static_cast<char *>((void*)&_packet), sizeof(_packet)),
                                  QHostAddress(HP_addr),
                                  port));
}

qint64 UDPService::sendPacket(int _address, int _parameter)
{
    return (sendPacket(CMD_ACT_PC_TO_PROBE, _address, _parameter));
}

qint64 UDPService::sendPacket(int _address)
{
    return sendPacket(_address, 0);
}

/*==================== EEPROM communication =================================================================*/
/**
 * UDP 패킷을 보내는 함수
 */
qint64 UDPService::sendPacket(unsigned char _cmd, void *pBuff, int Len)
{
    static unsigned char  _udpSendpacket[128];

    _udpSendpacket[0]  = HEADER;
    _udpSendpacket[1]  = HEADER;
    _udpSendpacket[2]  = _cmd;
    if (pBuff != nullptr && Len)
        memcpy((void*)&_udpSendpacket[3], pBuff, Len);
    _udpSendpacket[Len+3]  = 0;    //CheckSum
    _udpSendpacket[Len+4]  = TAIL;
    _udpSendpacket[Len+5]   = TAIL;

    return (socket->writeDatagram(QByteArray(static_cast<char *>((void*)_udpSendpacket), Len+6),
                                  QHostAddress(HP_addr),
                                  port));
}

qint64 UDPService::sendEEPROMReqPacket(void)
{
    return (sendPacket(CMD_EEPROM_REQUEST_PC_TO_HP, (void*)nullptr, 0));
}

qint64 UDPService::sendEEPROMWritePacket(char *pBuff)
{
    return (sendPacket(CMD_EEPROM_WRITE_PC_TO_HP, (void*)&pBuff[1], 10));
}
/*==================== EEPROM communication =================================================================*/

/***
 * Handpiece로 포지션 패킷을 보내는 함수
 * _position: 포지션 데이터
 * _nr: 포지션 갯수
 */
void UDPService::sendPositionPacket(tPosition* _position, uint8_t _nr)
{
    /* HEADER + HEADER2 + COMMAND + nr + TAIL + TAIL2 = 7 byte */
#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    char _positionPacket[7+POSTION_MAX*sizeof(tPosition)] = {0, };
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    unsigned char _positionPacket[7+POSTION_MAX*sizeof(tPosition)] = {0, };
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    uint16_t _xor_idx = 0;

    _positionPacket[0] = HEADER;
    _positionPacket[1] = HEADER;
    _positionPacket[2] = CMD_POSITION_PC_TO_PROBE;
    _positionPacket[3] = _nr;

    if (_nr)
        memcpy(_positionPacket+4, _position, sizeof(tPosition)*_nr);

    _xor_idx  = 3+_nr*sizeof(tPosition)+1;

    for (int i = 2; i < _xor_idx; i++)
    {
        _positionPacket[_xor_idx] ^= _positionPacket[i];
    }

    _positionPacket[_xor_idx+1] = TAIL;
    _positionPacket[_xor_idx+2] = TAIL;

    socket->writeDatagram(QByteArray(static_cast<char *>((void*)&_positionPacket), _xor_idx+3),
                                      QHostAddress(HP_addr),
                                      port);
}

/***
 * 파일의 페이로드를 전송하기 위한 함수
 */
void UDPService::_sendFilePayload(void* _data, uint64_t _length, uint64_t _fileSize)
{
    tFirmwarePacket _packet;
#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    char  Buf[sizeof(_packet)] = {0, };
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    unsigned char  Buf[sizeof(_packet)] = {0, };
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류

    _packet.Header  = HEADER;
    _packet.Header2 = HEADER;
    _packet.Command = CMD_UPDATE_DATA_PC_TO_PROBE;

    _packet.FileSize = _fileSize;
    _packet.PayloadSize = _length;
    memcpy(_packet.Payload, _data, _length);

#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    memcpy(&Buf, &_packet, sizeof(_packet));
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    memcpy(Buf, &_packet, sizeof(_packet));
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류

    _packet.CheckXOR = 0;

    for (uint16_t i = 2; i < 11+_length; i++)
    {
        _packet.CheckXOR ^= Buf[i];
    }

    if (_length < PAYLOAD_SIZE)
    {
        _packet.Payload[_length] = _packet.CheckXOR;
    }

    _packet.Tail  = TAIL;
    _packet.Tail2 = TAIL;

    socket->writeDatagram(QByteArray(static_cast<char *>((void*)&_packet), sizeof(_packet)),
                                      QHostAddress(HP_addr),
                                      port);
}

void _delay(unsigned int _i)
{
    volatile int i = _i;

    while(i--) {}
}

#include <QThread>
void UDPService::sendFile(QFile& _file)
{
    QFileInfo fileInfo(_file);
    qint64  _tmpSize = 0;

    qDebug() << "file size: " << fileInfo.size();

    while (!_file.atEnd())
    {
        QByteArray _data = _file.read(PAYLOAD_SIZE);

        _sendFilePayload(_data.data(), _data.size(), fileInfo.size());

        _tmpSize += _data.size();

        /* 전송의 안정성을 위해 일정시간 delay
         * Note: 시스템마다 틀릴수 있음 */
        //QThread::usleep(1);
        _delay(3000000);
    }

    if (fileInfo.size() != _tmpSize)
        qCritical() << "Firmware Size Error....";
}

/***
 * Handpiece의 상태를 요청하는 slot
 * 500ms마다 수행
 */
void UDPService::doRequestState()
{
    emit reqeustState();
}

void UDPService::onConnectError()
{
    connected = false;
    connectTimer->stop();

    emit handpieceError();
}


