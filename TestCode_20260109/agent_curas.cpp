#include "agent_curas.h"

#include "imageprocessor.h"
#include "QApplication"

/***
 * Agent CuRAS는 메인 역할을 하는
 * CuRAS 장비의 대리인 역할
 * inherited by SerailWorker(RS-232 Service)
 */
AgentCuRAS::AgentCuRAS(SerialWorker *parent) :
    SerialWorker(parent),
    Handpiece(new UDPService)
{
    initContext();

    connect(Handpiece,SIGNAL(readyPacket(uint8_t*, uint16_t)),this,SLOT(doHandpiecePacket(uint8_t*, uint16_t)), Qt::AutoConnection);
    connect(Handpiece,SIGNAL(reqeustState()),this,SLOT(doReqeustHandpieceState()));
#ifndef DISABLE_USART
    connect(Handpiece,SIGNAL(handpieceError()),this,SLOT(handpieceError()));
#endif
    connect(this,SIGNAL(readyPacket(QByteArray)),this,SLOT(processingPacket(QByteArray)));
#ifndef DISABLE_USART
    connect(this,SIGNAL(SIG_serial_error(bool)),this,SLOT(onSerialError(bool)));
#endif
    /* ========== EEPROM Tip JYH Append ========================================  */
    mEEPROM_Status = 0; //0x0101;   //0x0101, 0x0505, 0xFFFF;
    /* ========== EEPROM Tip JYH Append ========================================  */

    Handpiece->setStandby();

    this->inited = true;

    setUpdateDone();
}


AgentCuRAS::~AgentCuRAS()
{
    free(Handpiece);
}

void AgentCuRAS::initContext(void)
{
    this->inited = false;

    /* 상태 초기화 */
    context.ShotMode = -1;
    context.TargetTemp = -1;
    context.Status = 0;

    context.isUpdateMode = false;
    context.canUIupdate = false;

    this->calibration = false;
#ifdef DISABLE_USART
    context.SpotSize = 40;
#endif
    setStandby();


}

/* ========== EEPROM Tip JYH Append Test code ========================================  */
void AgentCuRAS::onTestTimeOver()
{
    //setTipState((int)100);
}
/* ========== EEPROM Tip JYH Append Test code ========================================  */

void AgentCuRAS::setUpdateDone(void)
{
    if (QCoreApplication::arguments().count() > 1)
    {
        if (QCoreApplication::arguments().at(1).compare("Program_Update_Success") == 0)
        {
            this->sendActionPacket(WRITE_PC_UPDATE_STATUS, UPDATE_DONE);
            qInfo() << "updated";
        }

        if (QCoreApplication::arguments().at(1).compare("Program_Update_Fail") == 0)
        {
            this->sendActionPacket(WRITE_PC_UPDATE_STATUS, UPDATE_ERROR);
            qInfo() << "failed";
        }

    }
}

/***
 * Handpiece에서 들어오는 패킷을 처리하는 slot
 *
 * 원래 Handpiece(udpservice) 객체에서 처리해야 하나,
 * 개발의 편의상 여기에서 처리
 */
 int Init_Send_Step = 0, Old_OPMode = 0;
void AgentCuRAS::doHandpiecePacket(uint8_t* _buffer, uint16_t _size)
{
    tPacket _packet;

    memcpy(&_packet, _buffer, sizeof(_packet));

    /* 만약 명령/상태 패킷이 아니면 */
    if (!isPacket(_packet))
    {
        /* Handpiece에서 오는 명령/상태가 아니면 */
        /* 이미지로 인식 */
        emit setImage(_buffer, _size);

        return;
    }

    switch (_packet.Command)
    {
    case SEND_ACTION:
    {
        switch (_packet.Address)
        {
#ifdef DISABLE_USART
        case SET_OPMODE:
            qDebug() << "SET_OPMODE : " << _packet.Address << " : " << _packet.Parameter;
            setOpMode(_packet.Parameter);
            break;
        case SET_SHOT_MODE:
            qDebug() << "from HP: " << _packet.Address << " : " << _packet.Parameter;
            switch(_packet.Parameter)
            {
                case 0 : setDetectModeState(SINGLE_NONE); break;
                case 3 : setDetectModeState(AUTO); break;
                case 5 : setDetectModeState(SEMIAUTO_WHITE); break;
                case 6 : setDetectModeState(MULTIPLE_6MM); break;
                default : setDetectModeState(_packet.Parameter); break;
            }
            break;
#else
        case SET_OPMODE:
        case SET_SHOT_MODE:
            qDebug() << "from HP: " << _packet.Address << " : " << _packet.Parameter;
            this->sendActionPacket(_packet.Address, _packet.Parameter);
            break;
#endif
        /* ========== 20221007 영상처리 관련 시작 패킷 추가 JYH Append ========================================  */
        case BEGIN_IMAGE:
            emit setImage(nullptr, 1);
            break;
        /* ========== 20221007 영상처리 관련 시작 패킷 추가 JYH Append ========================================  */
        case END_IMAGE:
            emit setImage(nullptr, 0);
            break;
        /***
         * 버전 Query에 대한 응답
         * called by getHandpieceVersion()
         */
        case GET_HANDPIECE_VERSION:
            qDebug() << "GET_HANDPIECE_VERSION from HP: " << _packet.Address << " : " << _packet.Parameter;
            this->sendActionPacket(WRITE_PROBE_VERSION_TO_CURAS, _packet.Parameter);
            break;
        case TOUCH_DETECT:
#ifdef DISABLE_TOUCH
            //qDebug() << "Touch DATA : " << _packet.Parameter;
            if(_packet.Parameter == 1 && context.OpMode == READY)
            {
                context.touchDetect = 0;
                //qDebug() << "Read Touch OFF";
            }
            else if(_packet.Parameter == 0 && context.OpMode == READY)
            {
                context.touchDetect = 1;
                //qDebug() << "Read Touch ON";
            }
#else
            context.touchDetect = _packet.Parameter;
            //qDebug() << "Touch DATA : " << _packet.Parameter;
#endif
            break;
        case SET_UPDATE_STATUS:
            doUpdateHandpiece(_packet.Parameter);
            break;
        case SET_UPDATE_PROGRESS:
            qDebug() << "SET_UPDATE_PROGRESS from HP: " << _packet.Parameter;
            this->sendActionPacket(WRITE_PROBE_UPDATE_PROGRESS, _packet.Parameter);
            break;
        default:
            qDebug() << "HP: Unknown Address " << _packet.Address;
        }
        break;
    }
    /***
     * 일정시간마다 수행되는 request에 대한 응답
     * 일정시간 응답이 없으면
     * 에러를 출력
     */
    case SEND_STATE:
        this->temperature = _packet.TempLow;
#ifdef DISABLE_USART
        if(Init_Send_Step < 4)
        {
            switch(Init_Send_Step)
            {
                case 0 :
                    context.GuiMode = TREATMENT_MODE;
                    Handpiece->sendPacket(SET_UI_MODE, context.GuiMode);
                    Init_Send_Step = 1;
                    break;
                case 1 :
                    //context.ShotMode = AUTO;
                    context.ShotMode = SEMIAUTO_WHITE;
                    setDetectModeState(context.ShotMode);
                    setShotMode(context.ShotMode);
                    Handpiece->sendPacket(SET_SHOT_MODE, SEMIAUTO_WHITE);
                    Init_Send_Step = 2;
                    break;
                case 2 :
                    Handpiece->sendPacket(CMD_EEPROM_REQUEST_PC_TO_HP, (void*)nullptr, 0);
                    Init_Send_Step = 3;
                    break;
                case 3 :
#ifdef DISABLE_TOUCH
                    //qDebug() << "Touch DATA : " << context.touchDetect; //1 터치
                    if(Old_OPMode != context.OpMode)
                    {
                        Old_OPMode = context.OpMode;
                        if(Old_OPMode == STANDBY)
                        {
                            //qDebug() << "AUTO STANDBY";
                            context.touchDetect = 0;
                        }
                        else if(Old_OPMode == READY)
                        {
                            //qDebug() << "AUTO READY";
                            context.touchDetect = 1;
                        }
                    }
#else
                    Init_Send_Step = 6;
#endif
                    break;
                default :
                    Init_Send_Step = 10;
                    break;
            }
        }
#endif
        break;
    /*==================== EEPROM communication =================================================================*/
    case CMD_EEPROM_HP_TO_PC:
        if (mEEPROM_Status != *((uint16_t*)&_buffer[3]))
        {
            mEEPROM_Status = *((uint16_t*)&_buffer[3]);
            setTipState((int)0);
        }
        //mEEPROM_Status = *((uint16_t*)&_buffer[3]);
        this->sendEEPROMPacket(_buffer[2], &_buffer[3]);
        break;
    /*==================== EEPROM communication =================================================================*/
    default:
        qDebug() << "HP Unknown Command";
        break;
    }
}


void AgentCuRAS::doUIupdate()
{
    QDir tmpDir = QDir("C:/VSLS_IMG");
    QString _filename = tmpDir.absolutePath() + "/swupdate.dat";
    QByteArray _bytes = QByteArray::fromHex("1");

    if (context.canUIupdate)
    {
        if (!tmpDir.exists())
            tmpDir.mkdir(".");

        QFile file(_filename);
        if (!file.open(QFile::WriteOnly))
        {
            qInfo() << "Could not open swupdate file for writing: " << _filename;
            return;
        }
        file.write(_bytes);
        file.close();

        this->sendActionPacket(WRITE_PC_UPDATE_STATUS, UPDATE_START);
    }

    context.isUpdateMode = false;
}

void AgentCuRAS::doUpdateHandpiece(int _parameter)
{
    context.isUpdateMode = true;

    switch (_parameter)
    {
    case UPDATE_READY:
        sendFirmware();
        qDebug() << "Update Ready and Send Firmware ...";
        break;
    case UPDATE_START:
        qDebug() << "Update Start...";
        break;
    case UPDATE_DONE:
        this->sendActionPacket(WRITE_PROBE_UPDATE_STATUS, UPDATE_DONE);
        qDebug() << "FW Update Done...";
        doUIupdate();
        break;
    case UPDATE_ERROR:
        qInfo() << "Update Error...";
        doUIupdate();
        break;
    }
}

void AgentCuRAS::sendFirmware()
{
    QString _filepath = SearchSwUpdateFile(this->HPFileName);

    qDebug() << _filepath;

    if (_filepath == "")
    {
        qInfo() << "Handpiece Firmware file not found...";
        return;
    }

    QFile _file(_filepath);
    if (!_file.open(QFile::ReadOnly))
    {
        qInfo() << "Could not open the firmware file...";
        return;
    }

    this->Handpiece->sendFile(_file);

    _file.close();
}


bool AgentCuRAS::isValidChecksum(tCuRASStatePacket& _CuRASStatePakcet, QByteArray& _pdata)
{
    unsigned char _xor = 0;

    for (int i = 0; i < (int)sizeof(tCuRASState); i++)
        _xor ^= ((unsigned char)_pdata[i]);
    return (_CuRASStatePakcet.CheckXOR == _xor);
}

bool AgentCuRAS::isValidChecksum(tPacket& _CuRASActionPacket, QByteArray& _pdata)
{
    unsigned char _xor = 0;

    for (int i = 0; i<9; i++)
        _xor ^= (unsigned char)_pdata[i];
    return (_CuRASActionPacket.CheckXOR == _xor);
}


/***
 * CuRAS에서 오는 패킷 처리
 */
void AgentCuRAS::processingPacket(QByteArray _pdata)
{
    tCuRASStatePacket   _CuRASStatePakcet;
    tPacket             _CuRASActionPacket;
    /* Command: _pdata->at(0) */
    switch((unsigned char)_pdata.at(0))
    {
    case CMD_CURAS_STAT_TO_PC:
        //qInfo("processingPacket(): CMD_CURAS_STAT_TO_PC : packet size=%d", _pdata.size()) ;
        memcpy(&_CuRASStatePakcet, _pdata.data(), _pdata.size());
        //qInfo("processingPacket(): CMD_CURAS_STAT_TO_PC : memcpy(&_CuRASStatePakcet, _pdata.data(), _pdata.size())") ;

        if (!isValidChecksum(_CuRASStatePakcet, _pdata))
            return;

        //qInfo("processingPacket(): CMD_CURAS_STAT_TO_PC : isValidChecksum(_CuRASStatePakcet, _pdata) ret true !!") ;

        setCuRASAllState(&_CuRASStatePakcet.CuRASState);
        //qInfo("processingPacket(): CMD_CURAS_STAT_TO_PC : setCuRASAllState(&_CuRASStatePakcet.CuRASState)") ;
        doResponseToCuRAS();
        //qInfo("processingPacket(): CMD_CURAS_STAT_TO_PC : doResponseToCuRAS()") ;
        break;

    case CMD_ACT_CURAS_TO_PC:
        memcpy(&_CuRASActionPacket.Command, _pdata.data(), _pdata.size());

        if (!isValidChecksum(_CuRASActionPacket, _pdata))
            return;

        doActionFromCuRAS(&_CuRASActionPacket);
        break;

    /*==================== EEPROM communication =================================================================*/
    case CMD_EEPROM_REQUEST_PC_TO_HP:
        this->Handpiece->sendEEPROMReqPacket();
        break;
    case CMD_EEPROM_WRITE_PC_TO_HP:
        this->Handpiece->sendEEPROMWritePacket((char*)_pdata.data());
        break;
    /*==================== EEPROM communication =================================================================*/
    default:
        {
            unsigned char *pDebug = (unsigned char*)_pdata.data();
            qInfo("CuRAS Unknown Command: %02x %02x %02x %02x Size=%d", pDebug[0], pDebug[1], pDebug[2], pDebug[3], _pdata.size());
        }
        break;
    }
}

void AgentCuRAS::doSWUpdate()
{
    QString _HP_FW = SearchSwUpdateFile(this->HPFileName);

    if (_HP_FW != "")
    {
        this->sendActionPacket(WRITE_PROBE_UPDATE_STATUS, UPDATE_START);
        Handpiece->sendPacket(SET_UPDATE_READY);
    }

    if (SearchSwUpdateFile(this->UIFileName) != "")
    {
        context.canUIupdate = true;

        if (_HP_FW == "")
            doUIupdate();
    }
}

void AgentCuRAS::doActionFromCuRAS(tPacket* _packet)
{
    switch (_packet->Address)
    {
    case SET_SHOT_MODE_TO_PC:
        qDebug() << "SET_SHOT_MODE_TO_PC: (do not implement...) ";
        break;
    case SET_STATUS:
        setFootSwitchState(_packet->Parameter);
        break;
    /***
     * MANUAL shot의 경우, Total Count
     */
    case SET_SHOT_COUNT:
        setShotCount(_packet->Parameter);
        break;
    case GET_HANDPIECE_VERSION:
        getHandpieceVersion();
        break;
    case GET_PC_VERSION:
        getAgentVersion();
        break;
    case SET_SW_UPDATE:
        doSWUpdate();
        qDebug() << "SET_SW_UPDATE: ";
        break;
    case SET_CALIBRATION_START:
        setCalibration(true);
        qDebug() << "WRITE_CMD_CAL_START_TO_PC";
        break;
    case SET_SYSTEM_SHOTDOWN:
        qDebug() << "SET_SYSTEM_SHOTDOWN: ";
        setSystemShotdown();
        break;
    default:
        qInfo("CuRAS: Unknown Address: %x", _packet->Address);
    }
}

#include <QProcess>
#include <QCoreApplication>

void AgentCuRAS::setSystemShotdown(void)
{
    QProcess process;
    QStringList arguments;
    arguments << "/s" << "/t" << "3";

    process.start("shutdown", arguments);
    process.waitForFinished(-1);

    QCoreApplication::quit();
}


#define CORRECT_POSITION_X(v) ((double)(CURAS_WIDTH * v) / IMAGE_WIDTH)
#define CORRECT_POSITION_Y(v) ((double)(CURAS_HEIGHT * v) / IMAGE_HEIGHT)
/***
 * CuRAS로 보정된 좌표를 보내는 함수
 */
void AgentCuRAS::sendPosition(vector<Point>& _points, Point _offset)
{
    tPosition _correct_positions[POSTION_MAX];

    if (_points.size() == 0 || this->getGUIMode() != TREATMENT_MODE)
        return;

    for(uint8_t _i = 0; _i < _points.size(); _i++)
    {
        _correct_positions[_i].x = (_points[_i].x + _offset.x);
        _correct_positions[_i].y = (_points[_i].y + _offset.y);

        _correct_positions[_i].x = CORRECT_POSITION_X(_correct_positions[_i].x);
        _correct_positions[_i].y = CORRECT_POSITION_Y(_correct_positions[_i].y);
        if (_i >= POSTION_MAX-1)
            qCritical() << " Overflow Postion size";
    }

    this->sendPositionPacket(_correct_positions, _points.size());
}

/*
 * Handpiece로 좌표를 보내는 함수
 */
void AgentCuRAS::sendPositionToHandpiece(vector<Point>& _points)
{
    static tPosition _positions[POSTION_MAX];

    if (!this->isSkinTouched())
        return;

    /* 포지션의 크기가 보낼 수 있는 크기보다 크면 에러 출력하고 리턴 */
    if (_points.size() > POSTION_MAX)
    {
        qCritical() << "sendPositionToHandpiece(): Position Size Overflow...";
        return;
    }

    for(uint8_t _i = 0; _i < _points.size(); _i++)
    {
        _positions[_i].x = _points[_i].x;
        _positions[_i].y = _points[_i].y;

        if (_i >= POSTION_MAX-1)
            qCritical() << " Overflow Postion size";
    }

    this->Handpiece->sendPositionPacket(_positions, _points.size());
}

/***
 * CuRAS의 주기적인 Query에 대해
 * 현재의 상태를 보내는 함수
 * 보내는값: Init상태, Handpiece연결상태, SkinTouch 센서 상태
 */
void AgentCuRAS::doResponseToCuRAS(void)
{
    static tStatus currentStatus;

    currentStatus.status = 0;
    currentStatus.init = this->isInited();
    currentStatus.hpConnected = getHanpiece()->isConnected();
    currentStatus.skinTouched = isSkinTouched();

    //qDebug("sendStatusPacket(temp=%d, status=%d)", this->temperature, currentStatus.status);
    sendStatusPacket(this->temperature, currentStatus.status);
}


/***
 * Standby 모드로 변경하고
 * Handpiece와 UI를 Standby 모드로 변경
 */
void AgentCuRAS::setStandby()
{
    context.OpMode = STANDBY;

    Handpiece->setStandby();
    emit SIG_setStandby();

    setCalibration(false);
    context.isUpdateMode = false;
    context.canUIupdate = false;

    qDebug() << "setStandby ";
}

/***
 * Ready 모드로 변경하고
 * Handpiece와 UI를 Ready 모드로 변경
 */
void AgentCuRAS::setReady()
{
    context.OpMode = READY;

    Handpiece->setReady();
    emit SIG_setReady();

    this->Handpiece->sendPacket(SET_SPOT_SIZE, getSpotRadiusSize());

    qDebug() << "setReady ";
}

/***
 * 동작모드를 변경하는 함수
 * 동작모드(OpMode): READY / STANDBY
 */
void AgentCuRAS::setOpMode(uint8_t _opMode)
{
    switch (_opMode)
    {
    case READY:
        setReady();
        break;
    case STANDBY:
    default:
        setStandby();
        break;
    }
}

/***
 * 환부의 Shot Mode를 설정
 */
void AgentCuRAS::setShotMode(int _shotMode)
{
    context.ShotMode = _shotMode;

    Handpiece->setShotmode(_shotMode);
    emit SIG_setShotmode(_shotMode);

    switch (_shotMode)
    {
    case SINGLE_NONE:
    case SINGLE_1HZ:
    case SINGLE_2HZ:
    case MULTIPLE_6MM:
    case MULTIPLE_10MM:
    case MULTIPLE_15MM:
    case MULTIPLE_20MM:
    case MULTIPLE_SQUARE_20MM:
        break;

    case SEMIAUTO_WHITE:
    case SEMIAUTO_GV:
    case AUTO:
        sendActionPacket(WRITE_AIMING_OFF, 1);
        break;
    }
}

/***
 * Spot Size : 반지름(radius)
 */
uint16_t AgentCuRAS::getSpotRadiusSize(void)
{
    uint16_t _SpotSize = getSpotSize();

    switch(this->getShotMode())
    {
    case MULTIPLE_6MM:
    case MULTIPLE_10MM:
    case MULTIPLE_15MM:
    case MULTIPLE_20MM:
    case MULTIPLE_SQUARE_20MM:
    case SINGLE_NONE:
    case SINGLE_1HZ:
    case SINGLE_2HZ:
    case SEMIAUTO_WHITE:
    case SEMIAUTO_GV:
    case AUTO:
        /* 30mm, 34mm, 40mm */
        if (_SpotSize == 30)
            return SPOT_3MM;
        else if (_SpotSize == 34)
            return SPOT_3_4MM;
        else if (_SpotSize == 40)
            return SPOT_4MM;
        break;
    }

    return 0;
}


/***
 * TEC 타겟 온도에 도달했는 지를 판단
 * NONE: 0
 * WAIT: 1
 */
void AgentCuRAS::setTargetWait(uint8_t _value)
{
    Handpiece->setTargetWait(_value);
    emit SIG_setTargetWait();

    qDebug() << "setTargetWait: " << _value;
}


void AgentCuRAS::setGUIMode(int _GUIMode)
{
    Handpiece->sendPacket(SET_UI_MODE, _GUIMode);
    qDebug() << "setGUIMode: " << _GUIMode;
}

void AgentCuRAS::setTargetTemp(int _value)
{
    Handpiece->setTargetTemp(_value);
    emit SIG_setTemperatureState();

    qDebug() << "setTargetTemp: " << _value;
}

void AgentCuRAS::setTargetOk(int _value)
{
    Handpiece->setTargetOk(_value);
    emit SIG_setTemperatureState();

    qDebug() << "setTargetOk : " << _value;
}

/* ========== EEPROM Tip JYH Append ========================================  */
void AgentCuRAS::setTipState(int _Percent)
{    //20221011 JYH Modify
    {
        context.TipPercent = _Percent;

        emit SIG_setTipState();

        //qDebug() << "setTipState : " << _Percent;
    }
}
/* ========== EEPROM Tip JYH Append ========================================  */

/***
 * Foot SW 상태가 준비되지 않으면
 * Standby 모드로...
 */
void AgentCuRAS::setFootEnNotReady(int _value)
{
    if (!_value)
        setStandby();
}


void AgentCuRAS::setFrequency(int _value)
{
    Q_UNUSED(_value);
    /* do not implement */
}


void AgentCuRAS::setCropPosition(int _value)
{
    this->getHanpiece()->sendPacket(SET_DISPLAY_CALIBRATION, _value);
    qDebug("setCropPosition: x = %d y = %d", _value>>16, _value&0xFFFF);
}

void AgentCuRAS::setBeamSize(uint8_t _value)
{
    Q_UNUSED(_value);
    /* do not implement */
}


void AgentCuRAS::setShotCount(uint8_t _value)
{
    context.shotOk = 1;
    context.shotCount = _value;
    Handpiece->sendPacket(SET_SHOT_OK, _value);

    if (!isSkinTouched() || getOpMode() != READY)
        return;

    switch (getShotMode())
    {
    case SINGLE_NONE:
    case SINGLE_1HZ:
    case SINGLE_2HZ:
        emit setShotOk();
        break;
    case MULTIPLE_6MM:
    case MULTIPLE_10MM:
    case MULTIPLE_15MM:
    case MULTIPLE_20MM:
    case MULTIPLE_SQUARE_20MM:
    case AUTO:
    case SEMIAUTO_WHITE:
    case SEMIAUTO_GV:
        emit setShotOk(_value);
        break;
    }

}


/***
 * Handpiece에게 물어보고 응답
 */
inline void AgentCuRAS::getHandpieceVersion(void)
{
    if (context.isUpdateMode)
        return;
    Handpiece->sendPacket(GET_HANDPIECE_VERSION);

    qDebug() << "Get Handpiece Version: ";
}


inline void AgentCuRAS::getAgentVersion(void)
{
    sendActionPacket(WRITE_PC_VERSION_TO_CURAS, PRODUCT_VERSION);
}



/***
 * CuRAS 장비의 현재상태를 저장
 */
void AgentCuRAS::setCuRASAllState(tCuRASState* _newState)
{
    setCuRASStatus(_newState);

    /* ========== EEPROM Tip JYH Append ========================================  */
    setTipState((int)_newState->TipPercent);
    /* ========== EEPROM Tip JYH Append ========================================  */
    setDetectModeState(_newState->ShotMode);
    setGUIModeState(_newState->GuiMode);
    setTargetTempState(_newState->TargetTemp);
    setFrequencyState(_newState->Frequency);
    setCropPostionState(_newState->CropPosition);
    setBeamSizeState(_newState->BeamSize);
    setSpotSize(_newState->SpotSize);
}

void AgentCuRAS::setCuRASStatus(tCuRASState* _newState)
{
    if (context.Status == _newState->Status)
        return;

    //qDebug() << "setCuRASStatus->UseMultiShotMode : " << _newState->UseMultiShotMode;

    setFootSwitchState(_newState);
    setOpModeState(_newState);
    setTargetOkState(_newState);
    setTargetWaitState(_newState);
    setFootEnNotReadyState(_newState);
    setUseMultiShotMode((uint8_t)_newState->UseMultiShotMode);  // 20240826 JYH Append
}


inline void AgentCuRAS::setFootSwitchState(tCuRASState* _state)
{
    if (context.FootSwitch != _state->FootSwitch)
    {
        context.FootSwitch = _state->FootSwitch;

#if 1       // 20221011 JYH Append
        if (!context.FootSwitch)
           setCalibration(false);
#endif       // 20221011 JYH Append

        Handpiece->sendPacket(SET_FOOTSWITCH, context.FootSwitch);

#if 1       // 20221011 JYH Append
        qDebug("1 changeFootSwitch=%d, shotCount=%d", context.FootSwitch, context.shotCount);
        context.shotCount = 0;
        if (!context.FootSwitch)    // && context.shotCount == 0)
        {
            setCalibration(false);
            sendActionPacket(WRITE_AIMING_OFF, 1);
            sendActionPacket(WRITE_IMAGE_PROCESSING_OK, 0);
        }
#endif       // 20221011 JYH Append

       //qDebug() << "changeFootSwitch: " << context.FootSwitch;
    }
}

inline void AgentCuRAS::setFootSwitchState(int _value)
{
    tCuRASState _newState;

    _newState.Status = _value;

    setFootSwitchState(&_newState);

    Handpiece->sendPacket(SET_FOOTSWITCH, _value);
    qDebug("2 changeFootSwitch=%d, shotCount=%d", context.FootSwitch, context.shotCount);
#if 1       // 20221011 JYH Append
    context.shotCount = 0;
    if (!_value)    // && context.shotCount == 0)
    {
        setCalibration(false);
        sendActionPacket(WRITE_AIMING_OFF, 1);
        sendActionPacket(WRITE_IMAGE_PROCESSING_OK, 0);
    }
#endif       // 20221011 JYH Append
}


inline void AgentCuRAS::setOpModeState(tCuRASState* _state)
{
    if (context.OpMode != _state->OpMode)
        setOpMode(_state->OpMode);
}

inline void AgentCuRAS::setTargetOkState(tCuRASState* _state)
{
    if (context.TargetOk != _state->TargetOk)
    {
        context.TargetOk = _state->TargetOk;
        setTargetOk(context.TargetOk);
    }
}


inline void AgentCuRAS::setTargetWaitState(tCuRASState* _state)
{
    if (context.TargetWait != _state->TargetWait)
    {
        context.TargetWait = _state->TargetWait;
        setTargetWait(context.TargetWait);
    }
}

inline void AgentCuRAS::setFootEnNotReadyState(tCuRASState* _state)
{
    if (context.FootEnNotReady != _state->FootEnNotReady)
    {
        context.FootEnNotReady = _state->FootEnNotReady;
        setFootEnNotReady(context.FootEnNotReady);
    }
}


inline void AgentCuRAS::setDetectModeState(int _detectMode)
{
    if (_isChanged(&context.ShotMode, _detectMode))
       setShotMode(_detectMode);
}

/***
 * CuRAS에서는 명령을 보내나
 * Handpiece에서는 처리하지 않음
 */
inline void AgentCuRAS::setGUIModeState(int _GUIMode)
{
    if (_isChanged(&context.GuiMode, _GUIMode))
        setGUIMode(_GUIMode);
}

inline void AgentCuRAS::setTargetTempState(int _value)
{
    if (_isChanged(&context.TargetTemp, _value))
        setTargetTemp(_value);
}


inline void AgentCuRAS::setFrequencyState(int _freq)
{
    if (_isChanged(&context.Frequency, _freq))
        setFrequency(_freq);
}

inline void AgentCuRAS::setCropPostionState(int _value)
{
    if (_isChanged(&context.CropPosition, _value))
        setCropPosition(_value);
}

inline void AgentCuRAS::setBeamSizeState(uint8_t _size)
{
    if (context.BeamSize != _size)
    {
        context.BeamSize = _size;
        setBeamSize(_size);
    }
}

inline void AgentCuRAS::setSpotSize(uint16_t _value)
{
    if (_isChanged(&context.SpotSize, _value))
    {
        this->Handpiece->sendPacket(SET_SPOT_SIZE, getSpotRadiusSize());
    }
}

// 20240826 JYH Append
inline void AgentCuRAS::setUseMultiShotMode(uint8_t _value)
{
    if (context.UseMultiShotMode !=_value)
    {
        context.UseMultiShotMode =_value;
        this->Handpiece->sendPacket(SET_USE_MULTIPLE_SHOT_MODE, context.UseMultiShotMode);
        emit SIG_setShotmode(context.ShotMode);
        qInfo() << "SET_USE_MULTIPLE_SHOT_MODE Change : " << context.UseMultiShotMode;
    }
}

/***
 * Handpiece에 에러가 발생하면
 * CuRAS를 Standby Mode로
 */
void AgentCuRAS::handpieceError(void)
{
#ifdef DISABLE_USART
    if (context.isUpdateMode)
        return;
    sendActionPacket(SET_OPMODE, STANDBY);
#endif
    qInfo() << "HandPiece Connction Error";
}

void AgentCuRAS::onSerialError(bool _value)
{
#ifdef DISABLE_USART
    Q_UNUSED(_value);

    if (context.isUpdateMode)
        return;

    setStandby();
#endif
    qInfo() << "CuRAS Conncetion Error";
}

/***
 * Handpiece의 상태를 요청하는 slot
 * 500ms마다 수행
 *
 * [중요] 현재 CuRAS의 상태도 같이 보내줌.
 *   Address   -> op_mode
 *   Parameter -> detect_mode
 */
void AgentCuRAS::doReqeustHandpieceState(void)
{
    Handpiece->sendPacket(CMD_PROBE_STAT_TO_PC, getOpMode(), getShotMode());
}


inline bool AgentCuRAS::isPacket(tPacket _packet)
{
    return ( !(_packet.Header != HEADER
            || _packet.Header2 != HEADER
            || _packet.Tail != TAIL
            || _packet.Tail2 != TAIL) ||
             /*==================== EEPROM communication =================================================================*/
             !(_packet.Header != HEADER
            || _packet.Header2 != HEADER
            || _packet.Command != CMD_EEPROM_HP_TO_PC));
            /*==================== EEPROM communication =================================================================*/
}


/**
 * return true -> changed
 */
bool AgentCuRAS::_isChanged(int* _orgValue, int _newValue)
{
    if (*_orgValue == _newValue)
        return false;

    *_orgValue = _newValue;

    return true;
}

/* ========== EEPROM Tip JYH Append ========================================  */
//bool AgentCuRAS::_isChanged(uint16_t* _orgValue, uint16_t _newValue)
bool AgentCuRAS::_isChanged(uint8_t* _orgValue, uint8_t _newValue)
/* ========== EEPROM Tip JYH Append ========================================  */
{
    if (*_orgValue == _newValue)
        return false;

    *_orgValue = _newValue;

    return true;
}

#include <QStorageInfo>
#include <QList>


QString AgentCuRAS::SearchSwUpdateFile(QString filename)
{
    QList<QStorageInfo> devices = QStorageInfo::mountedVolumes();

    foreach(QStorageInfo device, devices)
    {
        if (QDir(device.rootPath()).exists(filename))
            return device.rootPath() + filename;
    }

    return "";
}


