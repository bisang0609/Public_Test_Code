#ifndef AGENT_CURAS_H
#define AGENT_CURAS_H

#include "version.h"
#include "common.h"
#include "udpservice.h"
#include "serialworker.h"

#include <QLabel>
#include <QGraphicsWidget>
#include <QTimer>
#include <vector>

#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#define DISABLE_USART
//#define DISABLE_TOUCH
#define ADD_VIEW

#pragma pack(push, 1)
typedef struct {
    int GuiMode;
    int ShotMode;
    int TargetTemp;
    int Frequency;
    int CropPosition;   // int Overlap;
    uint8_t BeamSize;
    /* ========== EEPROM Tip JYH Append ========================================  */
    uint8_t SpotSize;
    uint8_t TipPercent;
    /* uint16_t SpotSize; */
    /* ========== EEPROM Tip JYH Append ========================================  */
    union {
        int Status;

        struct {
            uint8_t FootSwitch    : 1;
            uint8_t OpMode        : 1;
            uint8_t TargetOk      : 1;
            uint8_t TargetWait    : 1;
            uint8_t FootEnNotReady: 1;
            uint8_t ePopupOn      : 1;
            uint8_t    UseMultiShotMode   : 1;  //20240826 JYH Append
            // unsigned char dummy         : 2;
            // 원래 위에꺼 이나 Touch Detect를 저장하기 위해 아래와 같이 바꿈
            uint8_t touchDetect   : 1;
            uint8_t shotOk        : 1;

            uint8_t shotCount;
            bool    isUpdateMode  : 1;
            bool    canUIupdate   : 1;
        };
    };
} tCuRASState;

typedef struct  {
    unsigned char	Command;  	                    //  Define CMD_....
    tCuRASState     CuRASState;
    unsigned char	CheckXOR;
} tCuRASStatePacket;


typedef union {
    int status;

    struct {
        uint8_t init        : 1;
        uint8_t skinTouched : 1;
        uint8_t hpConnected : 1;
        uint8_t    dummy2 : 5;
        uint8_t    dummy1;
        uint16_t    dummy;
    };
} tStatus;
#pragma pack(pop)


class AgentCuRAS : public SerialWorker
{
    Q_OBJECT

public:
    explicit AgentCuRAS(SerialWorker *parent = 0);
    ~AgentCuRAS();

    /*** getter */
    inline uint8_t getOpMode(void)          { return context.OpMode; }
    inline uint8_t getShotMode(void)        { return context.ShotMode; }
    inline uint8_t getShotOk(void)          { return context.shotOk; }
    inline uint8_t getCropPosition(void)         { return context.CropPosition; }
    inline uint16_t getSpotSize(void)       { return context.SpotSize; }
    inline uint16_t getGUIMode(void)        { return context.GuiMode; }
    inline UDPService* getHanpiece(void)    { return this->Handpiece; }
    /* ========== EEPROM Tip JYH Append ========================================  */
    inline uint8_t getTipState(void)        { return context.TipPercent; }
    inline uint16_t getEEPROMStatus(void)        { return this->mEEPROM_Status; }
    /* ========== EEPROM Tip JYH Append ========================================  */

    inline uint8_t isTargetOk(void)         { return context.TargetOk; }
    inline int getTargetTemp(void)          { return context.TargetTemp; }
    inline uint8_t isTargetWait(void)       { return context.TargetWait; }
    inline uint8_t isFootSwitched(void)     { return context.FootSwitch; }

    inline uint8_t isSkinTouched(void)      { return context.touchDetect; }
    inline bool isInited(void)           { return this->inited; }
    inline bool isCalibration(void)         { return this->calibration; }
    inline void setCalibration(bool v)      { this->calibration = v; }

    inline uint16_t getUseMultiShotMode(void)   { return context.UseMultiShotMode; }

    void sendPosition(vector<Point>& _points, Point _offset);
    void sendPositionToHandpiece(vector<Point>& _points);

    uint16_t getSpotRadiusSize(void);

private:
    UDPService*     Handpiece;

    tCuRASState     context;        /* CuRAS의 상태를 관리하기 위한 컨텍스트 */
    int             temperature;    /* tCuRASState에 저장되어야 하나, 패킷처리로 따로 관리 */

    /* ========== EEPROM Tip JYH Append ========================================  */
    uint16_t mEEPROM_Status;
    QTimer* TestTimer;
    //void onTestTimeOver();  //Test code
    /* ========== EEPROM Tip JYH Append ========================================  */

    QString         HPFileName = "HP_FW.binary";
    QString         UIFileName = "VSLS_PC/VSLS_QTgui.exe";

    bool            inited;
    bool            calibration;

    void initContext(void);
    void setUpdateDone(void);
    void doSWUpdate();
    void doUIupdate();

    bool isPacket(tPacket _packet);
    bool isValidChecksum(tCuRASStatePacket& _CuRASStatePakcet, QByteArray& _pdata);
    bool isValidChecksum(tPacket& _CuRASActionPacket, QByteArray& _pdata);

    void setReady();
    void setStandby();

    void doActionFromCuRAS(tPacket* _packet);
    void doResponseToCuRAS(void);

    void setSystemShotdown(void);

    void doUpdateHandpiece(int _parameter);
    void sendFirmware();

    void setOpMode(uint8_t _opMode);
    void setTargetWait(uint8_t);
    void setShotMode(int);
    void setGUIMode(int);
    void setTargetTemp(int);
    void setTargetOk(int _value);
    /* ========== EEPROM Tip JYH Append ========================================  */
    void setTipState(int _Percent);
    /* ========== EEPROM Tip JYH Append ========================================  */

    void setFootEnNotReady(int _value);
    void setFrequency(int);
    void setCropPosition(int);
    void setBeamSize(uint8_t);
    void setShotCount(uint8_t);
    void getHandpieceVersion();
    void getAgentVersion();

    void setCuRASAllState(tCuRASState* _newState);
    void setCuRASStatus(tCuRASState* _newState);

    void setFootSwitchState(tCuRASState* _state);
    void setFootSwitchState(int _value);
    void setOpModeState(tCuRASState* _state);
    void setTargetOkState(tCuRASState* _state);
    void setTargetWaitState(tCuRASState* _state);
    void setFootEnNotReadyState(tCuRASState* _state);

    void setDetectModeState(int _detectMode);
    void setGUIModeState(int _GUIMode);
    void setTargetTempState(int _value);
    void setFrequencyState(int _freq);
    void setCropPostionState(int _value);
    void setBeamSizeState(uint8_t _size);
    void setSpotSize(uint16_t _value);
    void setUseMultiShotMode(uint8_t _value);  // 20240826 JYH Append

    bool _isChanged(int* _orgValue, int _newValue);
    /* ========== EEPROM Tip JYH Append ========================================  */
    bool _isChanged(uint8_t* _orgValue, uint8_t _newValue);
    //bool _isChanged(uint16_t* _orgValue, uint16_t _newValue);
    /* ========== EEPROM Tip JYH Append ========================================  */
    QString SearchSwUpdateFile(QString filename);


private slots:
    void doHandpiecePacket(uint8_t* _buffer, uint16_t _size);
    void processingPacket(QByteArray _pdata);

    void doReqeustHandpieceState();
    void handpieceError();
    void onSerialError(bool);
    /* ========== EEPROM Tip JYH Append ========================================  */
    void onTestTimeOver();  //Test code
    /* ========== EEPROM Tip JYH Append ========================================  */

signals:
    void setImage(uint8_t* _buffer, uint16_t _size);
    void SIG_setReady();
    void SIG_setStandby();
    void SIG_setShotmode(int);
    void SIG_setTargetWait();
    void SIG_setTemperatureState();
    /* ========== EEPROM Tip JYH Append ========================================  */
    void SIG_setTipState();
    /* ========== EEPROM Tip JYH Append ========================================  */
    void setShotOk();
    void setShotOk(uint8_t);
};

#endif // AGENT_CURAS_H
