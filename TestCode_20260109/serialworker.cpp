#include "serialworker.h"
#include <QtDebug>


SerialWorker::SerialWorker(SerialService *parent)
    : SerialService(parent)
{

}

SerialWorker::~SerialWorker()
{
}


bool SerialWorker::isHeader(uint8_t _firstHeader, uint8_t _secondHeader)
{
    return (_firstHeader == _secondHeader);
}

/***
 * CuRAS에서 오는 패킷을 파싱하는 함수
 * HEADER, TAIL : 각 2bytes
 * [HEADER와 TAIL을 제외한 Payload 부분만 파싱]
 */
void SerialWorker::parsingPacket(QByteArray* _data)
{
    static QByteArray   pdata;


    for(int i=0; i< _data->length(); i++)
    {
        switch ((unsigned char)_data->at(i))
        {
        case TAIL:
#if 0   // 20230321 JYH Modify 생산 이슈 PC와 Main 보드간 통신 안되는 문제
            if (pdata.size() > 0 && (unsigned char)pdata.at(0) != HEADER)
                emit readyPacket(pdata);

            pdata.clear();
#else   // 20230321 JYH Modify 생산 이슈 PC와 Main 보드간 통신 안되는 문제
            if (i > 0 && (unsigned char)_data->at(i-1) == TAIL)
            {
                pdata.remove(pdata.length()-1, 1);

                if (pdata.size() > 0 && (unsigned char)pdata.at(0) != HEADER)
                    emit readyPacket(pdata);

                pdata.clear();
            }
            else {
                pdata.append(_data->at(i));
            }
#endif   // 20230321 JYH Modify 생산 이슈 PC와 Main 보드간 통신 안되는 문제
            break;
        case HEADER:
            if (i > 0 && isHeader(_data->at(i-1), _data->at(i)))
                pdata.clear();
            else
                pdata.append(_data->at(i));
            break;
        default:
            pdata.append(_data->at(i));
            break;
        }
    }
}

/***
 * Action에 대한 패킷을 전송하는 함수
 * 오버로드되어 사용
 */
void SerialWorker::sendActionPacket(unsigned char _cmd, int _address, int _parameter)
{
    tPacket _packet;
#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    char  Buf[14] = {0, };
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    unsigned char  Buf[14] = {0, };
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류

    _packet.Header = HEADER;
    _packet.Header2 = HEADER;
    _packet.Command = _cmd;
    _packet.Address = _address;
    _packet.Parameter = _parameter;
    _packet.CheckXOR = 0;
    _packet.Tail = TAIL;
    _packet.Tail2 = TAIL;

#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    memcpy(&Buf, &_packet, sizeof(_packet));
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    memcpy(Buf, &_packet, sizeof(_packet));
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류

    for(int i=2; i<11; i++)
            Buf[11] ^= Buf[i];

    this->send((const char*)Buf, sizeof(_packet));
}

void SerialWorker::sendActionPacket(int _address, int _parameter)
{
    sendActionPacket(CMD_ACT_PC_TO_CURAS, _address, _parameter);
}

void SerialWorker::sendActionPacket(int _address)
{
    sendActionPacket(_address, 0);
}

/***
 * Status 패킷을 전송하는 함수
 */
void SerialWorker::sendStatusPacket(int _temperature, int _status)
{
    tStatusPacketToCuRAS _statusPacket;
#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    char  Buf[128] = {0, };
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    unsigned char  Buf[128] = {0, };
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류

    _statusPacket.Header = HEADER;
    _statusPacket.Header2 = HEADER;
    _statusPacket.Command = CMD_PC_STAT_TO_CURAS;
    _statusPacket.NowTemp = _temperature;
    _statusPacket.HpSerialNo = 255;         // FIXME
    _statusPacket.HpOperTime = 50;          // FIXME
    _statusPacket.Status = _status;
    _statusPacket.CheckXOR = 0;
    _statusPacket.Tail = TAIL;
    _statusPacket.Tail2 = TAIL;

#if 0   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    memcpy(&Buf, &_statusPacket, sizeof(_statusPacket));
#else   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류
    memcpy((void*)Buf, (void*)&_statusPacket, sizeof(_statusPacket));
#endif   // 20230321 JYH Modify 체크섬 char 형으로 계산 오류

    Buf[19] = 0;
    for(int i=2; i<19; i++)
        Buf[19] ^= Buf[i];

    this->send((const char*)Buf, sizeof(_statusPacket));

#if 0   // 20230320 JYH Append Debug Test Code
    QString sDebug;
    sDebug.sprintf("sendStatusPacket Data Size=19 :");
    for (int i = 0; i < 19; i++)
    {
        //qInfo("%02X ", CommData[i]);

        QString sTemp;
        sTemp.sprintf("%02X ", Buf[i]);
        //char sDebugBuff[512] = "";
        //sprintf(sDebugBuff, "%02X ", CommData[i]);
        sDebug.append(sTemp);
    }
    qInfo() << sDebug;
#endif   // 20230320 JYH Append Debug Test Code
}

/***
 * Postion 패킷을 전송하는 함수
 */
void SerialWorker::sendPositionPacket(tPosition* _position, uint8_t _nr)
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
    _positionPacket[2] = CMD_POSITION_PC_TO_CURAS;
    _positionPacket[3] = _nr;

    memcpy(_positionPacket+4, _position, sizeof(tPosition)*_nr);

    _xor_idx  = 3+_nr*sizeof(tPosition)+1;

    for (int i = 2; i < _xor_idx; i++)
    {
        _positionPacket[_xor_idx] ^= _positionPacket[i];
    }

    _positionPacket[_xor_idx+1] = TAIL;
    _positionPacket[_xor_idx+2] = TAIL;

    this->send((const char*)_positionPacket, _xor_idx+3);
}

/*==================== EEPROM communication =================================================================*/
/***
 * EEPROM 패킷을 전송하는 함수
 */
void SerialWorker::sendEEPROMPacket(unsigned char _cmd, unsigned char *_sROMData)
{
    /* HEADER + HEADER2 + COMMAND + STATUS(2) + DATA(8) + CHKSUM + TAIL + TAIL2 = 16 byte */
    unsigned char  Buf[16] = {0, };

    Buf[0] = HEADER;
    Buf[1] = HEADER;
    Buf[2] = _cmd;
    memcpy((void*)&Buf[3], (void*)_sROMData, 10);
    Buf[13] = 0;
    Buf[14] = TAIL;
    Buf[15] = TAIL;

    //qDebug("sendEEPROMPacket: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
    //       _sROMData[0], _sROMData[1], _sROMData[2], _sROMData[3], _sROMData[4], _sROMData[5], _sROMData[6], _sROMData[7], _sROMData[8], _sROMData[9]);

    for(int i=2; i<13; i++)
            Buf[13] ^= Buf[i];

    this->send((char*)Buf, 16);
}
/*==================== EEPROM communication =================================================================*/
