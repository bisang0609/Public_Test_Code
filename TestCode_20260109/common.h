#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define HEADER 	0xFA
#define TAIL 	0xFE

#define CMD_PROBE_STAT_TO_PC    0x31
#define CMD_POSITION_PC_TO_PROBE 0x32
#define CMD_ACT_PC_TO_PROBE     0x33
#define CMD_UPDATE_DATA_PC_TO_PROBE 0x34
#define CMD_ACT_CURAS_TO_PC     0x57
#define CMD_CURAS_STAT_TO_PC    0x71
#define CMD_PC_STAT_TO_CURAS    0x81
#define CMD_ACT_PC_TO_CURAS     0x86
#define CMD_POSITION_PC_TO_CURAS 0x87

//==================== EEPROM communication =================================================================//
#define CMD_EEPROM_REQUEST_PC_TO_HP      0x90
#define CMD_EEPROM_HP_TO_PC              0x91
#define CMD_EEPROM_WRITE_PC_TO_HP        0x92
//==================== EEPROM communication =================================================================//

#define SEND_ACTION				0x43
#define SEND_STATE              0x42

/**************** ADDRESS HANDPIECE *******************/
#define TOUCH_DETECT		1
#define SET_SHOT_OK             2
//#define CALL_FRAME              3       // do not use
#define SET_OPMODE              3       // OK
#define SET_SHOT_MODE           4
#define SET_UPDATE_STATUS       5
#define SET_UPDATE_PROGRESS 	6
#define SET_TARGET_TEMP         7
#define SET_TEC_CHECKING        8
#define SET_DISPLAY_CALIBRATION    9 //#define SET_OVERLAP             9
#define SET_UI_MODE             10

#define SET_TARGET_OK           11
#define SET_UPDATE_READY        12
#define SET_FOOTSWITCH          13
#define BEGIN_IMAGE             14
#define END_IMAGE               15

#define SET_BLUE_OV7725         16
#define SET_RED_OV7725          17
#define SET_GREEN_OV7725		18
#define SET_BRIGHTNESS_OV7725	19
#define SET_CONTRAST_OV7725 	20
#define SET_USAT_OV7725 		21
#define SET_VSAT_OV7725 		22
#define RESET_OV7725 			23

// KETI add
#define SET_GAIN_OV7725         24
#define SET_EXPOSURE_OV7725     25
#define SET_CAMERA_REGISTER     26


#define SET_SPOT_SIZE           31
#define SET_USE_MULTIPLE_SHOT_MODE 		32		// 20240826 JYH Append


enum {
    UPDATE_READY = 1,
    UPDATE_START = 2,
    UPDATE_DONE  = 3,
    UPDATE_ERROR = 4,
};


/****************ADDRESS FW*******************/
//#define WRITE_MOVE_OK_TO_PC				1       // do not used
//#define WRITE_SEND_READY_TO_CURAS       3       // do not used  /* merge */
//#define WRITE_SEND_SHOT_MODE_TO_CURAS   4       // do not used  /* merge */
#define WRITE_AIMING_OFF                5
#define WRITE_IMAGE_PROCESSING_OK       6
#define WRITE_PROBE_VERSION_TO_CURAS    7
#define WRITE_PC_VERSION_TO_CURAS       8
#define WRITE_PROBE_UPDATE_STATUS       9
#define WRITE_PROBE_UPDATE_PROGRESS     10
#define WRITE_PC_UPDATE_STATUS          11
#define SET_AIMING_CALIBRATION_FAIL      12
/*****************************************************/


/****************ADDRESS FW TO PC*******************/
#define SET_SHOT_MODE_TO_PC 	51
#define SET_STATUS 				52
#define SET_SHOT_COUNT          53
#define SET_CALIBRATION_START	54		//210823
#define GET_HANDPIECE_VERSION   55
#define GET_PC_VERSION          56
#define SET_SW_UPDATE           57
#define SET_SYSTEM_SHOTDOWN     58
/*****************************************************/

#define bitDepth     2   /* 2 */
#define IMAGE_WIDTH    240
#define IMAGE_HEIGHT   240
#define IMAGE_SIZE     IMAGE_WIDTH * IMAGE_HEIGHT * bitDepth
#define FRAME_SIZE     IMAGE_WIDTH * IMAGE_HEIGHT
/***********************
 * GUI MODE
 */
enum {
    HOME_MODE       = 1,
    TREATMENT_MODE  = 2,
    INFO_MODE       = 4,
    ERROR_LIST_MODE = 6,
    UPDATE_MODE     = 7
};

/***********************
 * OPERATION MODE
 */
enum {
    STANDBY = 0,
    READY,
    SHOT,
};

/***********************
 * SHOT MODE
 */
enum {
    NONE        = -1,
    SINGLE_NONE = 100,
    SINGLE_1HZ  = 101,
    SINGLE_2HZ  = 102,
    AUTO        = 103,
    SEMIAUTO_GV = 104,
    SEMIAUTO_WHITE  = 105,
    MULTIPLE_6MM    = 106,
    MULTIPLE_10MM   = 107,
    MULTIPLE_15MM   = 108,
    MULTIPLE_20MM   = 109,
    MULTIPLE_SQUARE_20MM = 110,
};

/***********************
 * TEMPERATURE
 */
enum {
    LOW_TEMPERATURE = 0,
    MID_TEMPERATURE,
    HIGH_TEMPERATURE,
};


enum {
    SPOT_3MM    = 18,
    SPOT_3_4MM  = 20,
    SPOT_4MM    = 22,
};

#pragma pack(push, 1)
typedef struct  {
    unsigned char   Header;		                    //  0xFA
    unsigned char   Header2;		                //  0xFA
    unsigned char	Command;  	                    //  Define CMD_....
    union {
        int	        Address;
        int         TempHigh;
    };
    union {
        int	        Parameter;
        int         TempLow;
    };
    unsigned char	CheckXOR;
    unsigned char   Tail;		                    //  0xFEFE
    unsigned char   Tail2;		                    //  0xFEFE
} tPacket;

typedef struct  {
    unsigned char   Header;		                    //  0xFA
    unsigned char   Header2;		                //  0xFA
    unsigned char	Command;  	                    //  Define CMD_....
    int				NowTemp;
    int				HpSerialNo;
    int				HpOperTime;
    int				Status;
    unsigned char	CheckXOR;
    unsigned char   Tail;		                    //  0xFEFE
    unsigned char   Tail2;		                    //  0xFEFE
} tStatusPacketToCuRAS;

/***
 * 포지션 구조체
 */
typedef struct {
    int16_t	y;
    int16_t	x;
} tPosition;

/***
 * Firmware Binary 전송을 위한 구조체
 */
#define PAYLOAD_SIZE    1400

typedef struct {
    uint8_t     Header;		                    //  0xFA
    uint8_t     Header2;		                //  0xFA
    uint8_t     Command;  	                    //  Define CMD_....
    uint32_t    FileSize;
    uint32_t    PayloadSize;
    uint8_t     Payload[PAYLOAD_SIZE];
    uint8_t     CheckXOR;
    uint8_t     Tail;		                    //  0xFEFE
    uint8_t     Tail2;		                    //  0xFEFE
} tFirmwarePacket;
#pragma pack(pop)


#define POSTION_MAX         250             /* 처리 및 전송할 수 있는 포지션의 최대 갯수 */

/* 영상이 출력되는 위젯의 크기 */
#define IMG_DISPLAY_WIDTH   960
#define IMG_DISPLAY_HEIGHT  960

/* CuRAS의 레이저 좌표 크기 */
#define CURAS_WIDTH         200
#define CURAS_HEIGHT        200

#define CENTER_X   (IMAGE_WIDTH/2)
#define CENTER_Y   (IMAGE_HEIGHT/2)


/***********************
 * Multiple 모드의 타겟 영역의 크기 (radius)
 */
enum {
    MULTIPLE_6MM_TARGET_RADIUS = 35,
    MULTIPLE_10MM_TARGET_RADIUS = 60,
    MULTIPLE_15MM_TARGET_RADIUS = 90,
    MULTIPLE_20MM_TARGET_RADIUS = 110,
};


#endif // COMMON_H
