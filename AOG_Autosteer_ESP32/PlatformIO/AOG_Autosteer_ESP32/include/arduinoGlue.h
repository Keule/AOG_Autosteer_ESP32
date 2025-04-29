#ifndef ARDUINOGLUE_H
#define ARDUINOGLUE_H


//============ Includes ====================
#include <Update.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <WiFi.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

//============ Defines & Macros====================
#define useLED_BUILTIN  0	          // some ESP board have a build in LED, some not. Here it's the same funtion as the WiFi LED
#define steerDataSentenceToAOGLengthV17 10
#define steerDataToAOGHeader  0xFD
#define steerDataFromAOGHeader  0xFE 
#define steerArdConfFromAOGHeader 0xFB
#define steerSettingsFromAOGHeader  0xFC
#define steerDataSentenceToAOGLength  14
#define incommingDataArraySize 5
#define SentenceFromAOGMaxLength 14
#define LED_WIFI_pulse 1000   //light on in ms 
#define LED_WIFI_pause 700    //light off in ms
#define EEPROM_SIZE 1024 //16. April 2021: 2x 250 needed
#define EE_ident1 0xED  // Marker Byte 0 + 1
#define ACTION_LoadDefaultVal   1
#define ACTION_RESTART          2
#define ACTION_SET_WAS_ZERO     3
#define ACTION_SET_INCL_ZERO    4
#define ACTION_SET_WS_THRESHOLD 5
#define BNO055_AOG_h
#define BNO055_CHIP_ID          0x00    // should be 0xA0              
#define BNO055_ACC_ID           0x01    // should be 0xFB              
#define BNO055_MAG_ID           0x02    // should be 0x32              
#define BNO055_GYRO_ID          0x03    // should be 0x0F              
#define BNO055_SW_REV_ID_LSB    0x04                                                                          
#define BNO055_SW_REV_ID_MSB    0x05
#define BNO055_BL_REV_ID        0x06
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_DATA_X_LSB   0x08
#define BNO055_ACC_DATA_X_MSB   0x09
#define BNO055_ACC_DATA_Y_LSB   0x0A
#define BNO055_ACC_DATA_Y_MSB   0x0B
#define BNO055_ACC_DATA_Z_LSB   0x0C
#define BNO055_ACC_DATA_Z_MSB   0x0D
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_MAG_DATA_X_MSB   0x0F
#define BNO055_MAG_DATA_Y_LSB   0x10
#define BNO055_MAG_DATA_Y_MSB   0x11
#define BNO055_MAG_DATA_Z_LSB   0x12
#define BNO055_MAG_DATA_Z_MSB   0x13
#define BNO055_GYR_DATA_X_LSB   0x14
#define BNO055_GYR_DATA_X_MSB   0x15
#define BNO055_GYR_DATA_Y_LSB   0x16
#define BNO055_GYR_DATA_Y_MSB   0x17
#define BNO055_GYR_DATA_Z_LSB   0x18
#define BNO055_GYR_DATA_Z_MSB   0x19
#define BNO055_EUL_HEADING_LSB  0x1A
#define BNO055_EUL_HEADING_MSB  0x1B
#define BNO055_EUL_ROLL_LSB     0x1C
#define BNO055_EUL_ROLL_MSB     0x1D
#define BNO055_EUL_PITCH_LSB    0x1E
#define BNO055_EUL_PITCH_MSB    0x1F
#define BNO055_QUA_DATA_W_LSB   0x20
#define BNO055_QUA_DATA_W_MSB   0x21
#define BNO055_QUA_DATA_X_LSB   0x22
#define BNO055_QUA_DATA_X_MSB   0x23
#define BNO055_QUA_DATA_Y_LSB   0x24
#define BNO055_QUA_DATA_Y_MSB   0x25
#define BNO055_QUA_DATA_Z_LSB   0x26
#define BNO055_QUA_DATA_Z_MSB   0x27
#define BNO055_LIA_DATA_X_LSB   0x28
#define BNO055_LIA_DATA_X_MSB   0x29
#define BNO055_LIA_DATA_Y_LSB   0x2A
#define BNO055_LIA_DATA_Y_MSB   0x2B
#define BNO055_LIA_DATA_Z_LSB   0x2C
#define BNO055_LIA_DATA_Z_MSB   0x2D
#define BNO055_GRV_DATA_X_LSB   0x2E
#define BNO055_GRV_DATA_X_MSB   0x2F
#define BNO055_GRV_DATA_Y_LSB   0x30
#define BNO055_GRV_DATA_Y_MSB   0x31
#define BNO055_GRV_DATA_Z_LSB   0x32
#define BNO055_GRV_DATA_Z_MSB   0x33
#define BNO055_TEMP             0x34
#define BNO055_CALIB_STAT       0x35
#define BNO055_ST_RESULT        0x36
#define BNO055_INT_STATUS       0x37
#define BNO055_SYS_CLK_STATUS   0x38
#define BNO055_SYS_STATUS       0x39
#define BNO055_SYS_ERR          0x3A
#define BNO055_UNIT_SEL         0x3B
#define BNO055_OPR_MODE         0x3D
#define BNO055_PWR_MODE         0x3E
#define BNO055_SYS_TRIGGER      0x3F
#define BNO055_TEMP_SOURCE      0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_ACC_RADIUS_MSB   0x68
#define BNO055_MAG_RADIUS_LSB   0x69
#define BNO055_MAG_RADIUS_MSB   0x6A
#define BNO055_PAGE_ID          0x07
#define BNO055_ACC_CONFIG       0x08
#define BNO055_MAG_CONFIG       0x09
#define BNO055_GYRO_CONFIG_0    0x0A
#define BNO055_GYRO_CONFIG_1    0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK          0x0F
#define BNO055_INT_EN           0x10
#define BNO055_ACC_AM_THRES     0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION  0x13
#define BNO055_ACC_HG_THRESH    0x14
#define BNO055_ACC_NM_THRESH    0x15
#define BNO055_ACC_NM_SET       0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET     0x18
#define BNO055_GYR_DUR_X        0x19
#define BNO055_GYR_HR_Y_SET     0x1A
#define BNO055_GYR_DUR_Y        0x1B
#define BNO055_GYR_HR_Z_SET     0x1C
#define BNO055_GYR_DUR_Z        0x1D
#define BNO055_GYR_AM_THRESH    0x1E
#define BNO055_GYR_AM_SET       0x1F
#define NUM_BNO055_OFFSET_REGISTERS (22)
#define BNO080_DEFAULT_ADDRESS 0x4B
#define I2C_BUFFER_LENGTH BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 32
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_GET_FEATURE_RESPONSE 0xFC //Add by Math : Get Feature Response report
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11
#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5
#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)
#define MMA8452_H_
#define MMA_ADDRESS 0x1C
#define STATUS        0x00  // Real time status
#define OUT_X_MSB     0x01  // [7:0] are 8 MSBs of 12-bit sample
#define OUT_X_LSB     0x02  // [7:4] are 4 LSBs of 12-bit sample
#define OUT_Y_MSB     0x03  // [7:0] are 8 MSBs of 12-bit sample
#define OUT_Y_LSB     0x04  // [7:4] are 4 LSBs of 12-bit sample
#define OUT_Z_MSB     0x05  // [7:0] are 8 MSBs of 12-bit sample
#define OUT_Z_LSB     0x06  // [7:4] are 4 LSBs of 12-bit sample
#define FIFO_SETUP    0x09  // FIFO Setup (8451 only)
#define SYSMOD        0x0B  // Current System Mode
#define INT_SOURCE      0x0C  // Interrupt status
#define WHO_AM_I      0x0D  // Device ID (0x2A)
#define XYZ_DATA_CFG    0x0E  // HPF Data Out and Dynamic Range Settings
#define HP_FILTER_CUTOFF  0x0F  // Cutoff frequency is set to 16 Hz @ 800 Hz
#define PL_STATUS     0x10  // Landscape/Portrait orientation status
#define PL_CFG        0x11  // Landscape/Portrait configuration
#define PL_COUNT      0x12  // Landscape/Portrait debounce counter
#define PL_BF_ZCOMP     0x13  // Back-Front, Z-Lock Trip threshold
#define P_L_THS_REG     0x14  // Portrait to Landscape Trip Angle is 29°
#define FF_MT_CFG     0x15  // Freefall/Motion functional block configuration
#define FF_MT_SRC     0x16  // Freefall/Motion event source register
#define FF_MT_THS     0x17  // Freefall/Motion threshold register
#define FF_MT_COUNT     0x18  // Freefall/Motion debounce counter
#define TRANSIENT_CFG   0x1D  // Transient functional block configuration
#define TRANSIENT_SRC   0x1E  // Transient event status register
#define TRANSIENT_THS   0x1F  // Transient event threshold
#define TRANSIENT_COUNT   0x20  // Transient debounce counter
#define PULSE_CFG     0x21  // ELE, Double_XYZ or Single_XYZ
#define PULSE_SRC     0x22  // EA, Double_XYZ or Single_XYZ
#define PULSE_THSX      0x23  // X pulse threshold
#define PULSE_THSY      0x24  // Y pulse threshold
#define PULSE_THSZ      0x25  // Z pulse threshold
#define PULSE_TMLT      0x26  // Time limit for pulse
#define PULSE_LTCY      0x27  // Latency time for 2nd pulse
#define PULSE_WIND      0x28  // Window time for 2nd pulse
#define ASLP_COUNT      0x29  // Counter setting for Auto-SLEEP
#define CTRL_REG1     0x2A  // Data Rate, ACTIVE Mode
#define CTRL_REG2     0x2B  // Sleep Enable, OS Modes, RST, ST
#define CTRL_REG3     0x2C  // Wake from Sleep, IPOL, PP_OD
#define CTRL_REG4     0x2D  // Interrupt enable register
#define CTRL_REG5     0x2E  // Interrupt pin (INT1/INT2) map
#define OFF_X       0x2F  // X-axis offset adjust
#define OFF_Y       0x30  // Y-axis offset adjust
#define OFF_Z       0x31  // Z-axis offset adjust
	#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
	#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
	#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
	#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
	#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND
    #define ADS1115_REG_POINTER_CONVERT     (0x00)
    #define ADS1115_REG_POINTER_CONFIG      (0x01)
    #define ADS1115_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS1115_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS1115_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS1115_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS1115_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS1115_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3
    #define ADS1115_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS1115_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS1115_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS1115_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS1115_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS1115_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16
    #define ADS1115_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)
    #define ADS1115_REG_CONFIG_DR_8SPS    (0x0000)  // 8 SPS(Sample per Second), or a sample every 125ms
    #define ADS1115_REG_CONFIG_DR_16SPS    (0x0020)  // 16 SPS, or every 62.5ms
    #define ADS1115_REG_CONFIG_DR_32SPS    (0x0040)  // 32 SPS, or every 31.3ms
    #define ADS1115_REG_CONFIG_DR_64SPS    (0x0060)  // 64 SPS, or every 15.6ms
    #define ADS1115_REG_CONFIG_DR_128SPS   (0x0080)  // 128 SPS, or every 7.8ms  (default)
    #define ADS1115_REG_CONFIG_DR_250SPS   (0x00A0)  // 250 SPS, or every 4ms, note that noise free resolution is reduced to ~14.75-16bits, see table 2 in datasheet
    #define ADS1115_REG_CONFIG_DR_475SPS   (0x00C0)  // 475 SPS, or every 2.1ms, note that noise free resolution is reduced to ~14.3-15.5bits, see table 2 in datasheet
    #define ADS1115_REG_CONFIG_DR_860SPS   (0x00E0)  // 860 SPS, or every 1.16ms, note that noise free resolution is reduced to ~13.8-15bits, see table 2 in datasheet
#define DEBUG_PING(...) Serial.printf(__VA_ARGS__)
#define DEBUG_PING(...)

//============ Structs, Unions & Enums ============
//-- from zAOG_ping.h
struct ping_option {
    uint32_t count;
    uint32_t ip;
    uint32_t coarse_time;
    ping_recv_function recv_function;
    ping_sent_function sent_function;
    void* reverse;
};

//-- from zAOG_ping.h
struct ping_resp {
    uint32_t total_count;
    float resp_time;
    uint32_t seqno;
    uint32_t timeout_count;
    uint32_t bytes;
    uint32_t total_bytes;
    float total_time;
    int8_t  ping_err;
};

//-- from MMA8452_AOG.h
typedef enum {
	MMA_RANGE_2G = 0,
	MMA_RANGE_4G,
	MMA_RANGE_8G
} mma8452_range_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_STANDBY = 0,
	MMA_WAKE,
	MMA_SLEEP
} mma8452_mode_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_HP1 = 0,
	MMA_HP2,
	MMA_HP3,
	MMA_HP4
} mma8452_highpass_mode_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_PORTRAIT_UP = 0,
	MMA_PORTRAIT_DOWN,
	MMA_LANDSCAPE_RIGHT,
	MMA_LANDSCAPE_LEFT
} mma8452_orientation_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_FREEFALL = 0,
	MMA_MOTION
} mma8452_motion_type_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_SLEEP_50hz = 0,
	MMA_SLEEP_12_5hz,
	MMA_SLEEP_6_25hz,
	MMA_SLEEP_1_56hz
} mma8452_sleep_frequency_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_800hz = 0,
	MMA_400hz,
	MMA_200hz,
	MMA_100hz,
	MMA_50hz,
	MMA_12_5hz,
	MMA_6_25hz,
	MMA_1_56hz
} mma_datarate_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_NORMAL = 0,
	MMA_LOW_NOISE_LOW_POWER,
	MMA_HIGH_RESOLUTION,
	MMA_LOW_POWER
} mma_power_mode_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_AUTO_SLEEP = 0x80,
	MMA_TRANSIENT = 0x20,
	MMA_ORIENTATION_CHANGE = 0x10,
	MMA_TAP = 0x08,
	MMA_FREEFALL_MOTION = 0x04,
	MMA_DATA_READY = 0x01
} mma_interrupt_types_t;

//-- from MMA8452_AOG.h
typedef enum {
	MMA_X = 0x01,
	MMA_Y = 0x02,
	MMA_Z = 0x04,
	MMA_ALL_AXIS = 0x07
} mma_axis_t;

//-- from BNO055_AOG.h
typedef struct
{
  uint16_t accel_offset_x;
  uint16_t accel_offset_y;
  uint16_t accel_offset_z;
  uint16_t gyro_offset_x;
  uint16_t gyro_offset_y;
  uint16_t gyro_offset_z;
  uint16_t mag_offset_x;
  uint16_t mag_offset_y;
  uint16_t mag_offset_z;

  uint16_t accel_radius;
  uint16_t mag_radius;
} bno055_offsets_t;

//-- from AOG_Autosteer_ESP32.ino
struct Storage {
	//WiFi
	char ssid1[24] = "Fendt_209V";                // WiFi network Client name
	char password1[24] = "";                      // WiFi network password
	char ssid_ap[24] = "Autosteer_unit_Net";	  // name of Access point, if no WiFi found, NO password!!
	uint16_t timeoutRouter = 120;                 // time (seconds) to wait for WIFI access, after that own Access Point starts
	byte timeoutWebIO = 255;                      // time (min) afterwards webinterface is switched off

	byte WiFi_myip[4] = { 192, 168, 1, 77 };      // autosteer module 
	byte WiFi_gwip[4] = { 192, 168, 1, 1 };       // Gateway IP only used if Accesspoint created
	byte WiFi_ipDest_ending = 255;                // ending of IP address to send UDP data to
	byte mask[4] = { 255, 255, 255, 0 };
	byte myDNS[4] = { 8, 8, 8, 8 };               //optional

	//Ethernet
	byte Eth_myip[4] = { 192, 168, 1, 78 };       // autosteer module 
	byte Eth_ipDest_ending = 255;                 // ending of IP address to send UDP data to
	byte Eth_mac[6] = { 0x70,0x69,0x69,0x2D,0x30,0x31 };
	bool Eth_static_IP = false;					          // false = use DHPC and set last number to 80 (x.x.x.80) / true = use IP as set above

	unsigned int PortAutostToAOG = 5577;          // this is port of this module: Autosteer = 5577 IMU = 5566 GPS = 
	unsigned int PortFromAOG = 8888;              // port to listen for AOG
	unsigned int PortDestination = 9999;          // port of AOG that listens

	//general settings
	uint8_t aogVersion = 20;			                // Version number for version check 4.3.10 = 4+3+10 = 17	

	byte DataTransVia = 7;                        // transfer data via 0 = USB / 7 = WiFi UDP / 10 = Ethernet UDP

	uint8_t output_type = 2;                      // set to 1  if you want to use Stering Motor + Cytron MD30C Driver
																// set to 2  if you want to use Stering Motor + IBT 2  Driver
																// set to 3  if you want to use IBT 2  Driver + PWM 2-Coil Valve
																// set to 4  if you want to use  IBT 2  Driver + Danfoss Valve PVE A/H/M


	uint16_t PWMOutFrequ = 20000;                 // PWM frequency for motordriver: 1000Hz:for low heat at PWM device 20000Hz: not hearable

	uint8_t	MotorDriveDirection = 0;              // 0 = normal, 1 = inverted

	uint8_t MotorSlowDriveDegrees = 5;	          // How many degrees before decreasing Max PWM

	uint8_t WASType = 2;                          // 0 = No ADS installed, Wheel Angle Sensor connected directly to ESP at GPIO 36 (pin set below) (attention 3,3V only)
																// 1 = Single Mode of ADS1115 - Sensor Signal at A0 (ADS)
																// 2 = Differential Mode - Connect Sensor GND to A1, Signal to A0

	uint8_t IMUType = 0;                          // 0: none, 1: BNO055 IMU, 2: CMPS14, 3: BNO080 + BNO085

	//CMPS14	
	int CMPS14_ADDRESS = 0x60;                    // Address of CMPS14 shifted right one bit for arduino wire library
	float CMPS14HeadingCorrection = 0.0;		      // not used at the moment
	float CMPS14RollCorrection = 0.0;		          // not used at the moment
	uint8_t CMPS14UsedAxis = 0;		                // not used at the moment

	// BNO08x
	uint8_t bno08xAddresses[2] = { 0x4A,0x4B };	  // BNO08x address variables to check where it is
	float BNOHeadingCorrection = 0.0;		          // not used at the moment
	float BNORollCorrection = 0.0;		            // not used at the moment
	uint8_t BNOUsedAxis = 0;		                  // not used at the moment

	//MMA
	uint8_t MMAInstalled = 0;                     // set to 1 if MMA8452 is installed at address 1C (Adr PIN to GND) set to 2 at address 1D (Adr PIN open)
	uint8_t UseMMA_X_Axis = 1;		                // 1: use X axis (default) 0: use Y axis
	uint8_t MMA_roll_MAX_STEP = 10;		            // max roll step per loop (5-20) higher = filter less

	uint8_t InvertRoll = 0;                       // 0: no, set to 1 to change roll direction
	uint8_t InvertWAS = 0;                        // set to 1 to Change Direction of Wheel Angle Sensor - to + 

	uint8_t ShaftEncoder = 0;                     // Steering Wheel ENCODER Installed
	uint8_t PressureSensor = 0;		        // (not supported at the moment)
	uint8_t CurrentSensor = 0;		        // (not supported at the moment)
	uint8_t pulseCountMax = 3;                    // Switch off Autosteer after x Pulses from Steering wheel encoder 

	uint16_t WebIOSteerPosZero = 10300;	          // first value for steer zero position ADS: 11000 for EPS32 AD PIN: 2048

	uint8_t AckermanFix = 78;		                  // if values for left and right are the same: 100 

	uint8_t SteerSwitchType = 1;                  // 0 = enable = switch high (3,3V) //1 = enable = switch low(GND) //2 = toggle = button to low(GND)
																// 3 = enable = button to high (3,3V), disable = button to low (GND), neutral = 1,65V
													// 255 = no steer switch, allways on if AOG steering is active

	uint8_t WorkSW_mode = 2;                      // 0 = disabled // 1 = digital ON/OFF // 2 = analog Value 0...4095 (0 - 3,3V)

	uint8_t Invert_WorkSW = 0;                    // 0 = Hitch raised -> High    // 1 = Hitch raised -> Low

	float autoSteerMinSpeed = 0.2;                // Min speed to use autosteer km/h
	float autoSteerMaxSpeed = 30;                 // Max speed to use autosteer km/h

	uint16_t WorkSW_Threshold = 1600;             // Value for analog hitch level to switch workswitch  (0-4096)


	// IO pins ------------------------------------------------------------------

	// set to 255 for unused !!!!!
	uint8_t SDA = 21;	                  // I2C Pins
	uint8_t SCL = 22;

	uint8_t AutosteerLED_PIN = 2;       // light on active autosteer and IBT2
	uint8_t LEDWiFi_PIN = 0;            // light on WiFi connected, flashes on searching Network. If GPIO 0 is used LED must be activ LOW otherwise ESP won't boot
	uint8_t LEDWiFi_ON_Level = LOW;	    // HIGH = LED on high, LOW = LED on low

										// (not supported at the moment) relais for section control
	uint8_t Relay_PIN[16] = { 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255 }; 
	uint8_t Tram_PIN[3] = { 255,255,255 };  // (not supported at the moment) relais for tramline control
	uint8_t Relays_ON = HIGH;		        // HIGH = Relay on high, LOW = Relay on low

	uint8_t WAS_PIN = 36;               // PIN for Wheel Angle Sensor (none, if ADS used)
	uint8_t WAS_Diff_GND_PIN = 39;
	uint8_t WORKSW_PIN = 33;            // PIN for workswitch (can be analog or on/off switch see WorkSW_mode)
	uint8_t STEERSW_PIN = 34;           // Pin for steer button or switch (see SteerSwitchType)
	uint8_t encA_PIN = 4;               // Pin for steer encoder, to turn off autosteer if steering wheel is used
	uint8_t encB_PIN = 32;              // Pin for steer encoder, to turn off autosteer if steering wheel is used

	uint8_t Servo_PIN = 16;			        // (not supported at the moment) Pin for servo to pull motor to steering wheel

	uint8_t PWM_PIN = 27;               // PWM Output to motor controller (IBT2 or cytron)
	uint8_t DIR_PIN = 26;               // direction output to motor controller (IBT2 or cytron)
	uint8_t Current_sens_PIN = 35;	    // (not supported at the moment) current sensor for IBT2 to read the force needed to turn steering wheel

	uint8_t Eth_CS_PIN = 5;             // CS PIN with SPI Ethernet hardware  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5

	uint8_t CAN_RX_PIN = 25;		        // (not supported at the moment) CAN bus 
	uint8_t CAN_TX_PIN = 17;		        // (not supported at the moment)

	//##########################################################################################################
	//### End of Setup Zone ####################################################################################
	//##########################################################################################################
	//filter variables set by AOG via PGN settings sentence
	float Ko = 0.05f;     //overall gain  
	float Kp = 20.0f;     //proportional gain  
	float Ki = 0.001f;    //integral gain
	float Kd = 1.0f;      //derivative gain 
	float AOGSteerPositionZero = 0;
	float steerSensorCounts = 100;
	uint16_t roll_corr = 200;
	byte minPWM = 40, highPWM = 150, lowPWM = 60;

	bool debugmode = false;
	bool debugmodeDataFromAOG = false;

};

//============ Extern Variables ============
extern float           AOGSteerPositionZero;              		//-- from AOG_Autosteer_ESP32
extern uint8_t         AckermanFix;                       		//-- from AOG_Autosteer_ESP32
extern int             AnalogValue;                       		//-- from AOG_Autosteer_ESP32
extern uint8_t         AutosteerLED_PIN;                  		//-- from AOG_Autosteer_ESP32
extern float           BNOHeadingCorrection;              		//-- from AOG_Autosteer_ESP32
extern float           BNORollCorrection;                 		//-- from AOG_Autosteer_ESP32
extern uint8_t         BNOUsedAxis;                       		//-- from AOG_Autosteer_ESP32
extern uint8_t         CAN_RX_PIN;                        		//-- from AOG_Autosteer_ESP32
extern uint8_t         CAN_TX_PIN;                        		//-- from AOG_Autosteer_ESP32
extern float           CMPS14HeadingCorrection;           		//-- from AOG_Autosteer_ESP32
extern float           CMPS14RollCorrection;              		//-- from AOG_Autosteer_ESP32
extern uint8_t         CMPS14UsedAxis;                    		//-- from AOG_Autosteer_ESP32
extern int             CMPS14_ADDRESS;                    		//-- from AOG_Autosteer_ESP32
extern uint8_t         CurrentSensor;                     		//-- from AOG_Autosteer_ESP32
extern uint8_t         Current_sens_PIN;                  		//-- from AOG_Autosteer_ESP32
extern uint8_t         DIR_PIN;                           		//-- from AOG_Autosteer_ESP32
extern byte            DataToAOGLength;                   		//-- from AOG_Autosteer_ESP32
extern byte            DataTransVia;                      		//-- from AOG_Autosteer_ESP32
extern boolean         EEPROM_clear;                      		//-- from AOG_Autosteer_ESP32
extern EthernetUDP     EthUDPFromAOG;                     		//-- from AOG_Autosteer_ESP32
extern bool            EthUDPRunning;                     		//-- from AOG_Autosteer_ESP32
extern EthernetUDP     EthUDPToAOG;                       		//-- from AOG_Autosteer_ESP32
extern uint8_t         Eth_CS_PIN;                        		//-- from AOG_Autosteer_ESP32
extern byte            Eth_connect_step;                  		//-- from AOG_Autosteer_ESP32
extern byte            Eth_ipDest_ending;                 		//-- from AOG_Autosteer_ESP32
extern IPAddress       Eth_ipDestination;                 		//-- from AOG_Autosteer_ESP32
extern bool            Eth_static_IP;                     		//-- from AOG_Autosteer_ESP32
extern float           G;                                 		//-- from AOG_Autosteer_ESP32
extern uint8_t         IMUType;                           		//-- from AOG_Autosteer_ESP32
extern uint8_t         InvertRoll;                        		//-- from AOG_Autosteer_ESP32
extern uint8_t         InvertWAS;                         		//-- from AOG_Autosteer_ESP32
extern uint8_t         Invert_WorkSW;                     		//-- from AOG_Autosteer_ESP32
extern float           Kd;                                		//-- from AOG_Autosteer_ESP32
extern float           Ki;                                		//-- from AOG_Autosteer_ESP32
extern float           Ko;                                		//-- from AOG_Autosteer_ESP32
extern float           Kp;                                		//-- from AOG_Autosteer_ESP32
extern uint8_t         LEDWiFi_ON_Level;                  		//-- from AOG_Autosteer_ESP32
extern uint8_t         LEDWiFi_PIN;                       		//-- from AOG_Autosteer_ESP32
extern boolean         LED_WIFI_ON;                       		//-- from AOG_Autosteer_ESP32
extern MMA8452         MMA1C;                             		//-- from AOG_Autosteer_ESP32
extern uint8_t         MMAInstalled;                      		//-- from AOG_Autosteer_ESP32
extern uint8_t         MMA_roll_MAX_STEP;                 		//-- from AOG_Autosteer_ESP32
extern uint8_t         MotorDriveDirection;               		//-- from AOG_Autosteer_ESP32
extern uint8_t         MotorSlowDriveDegrees;             		//-- from AOG_Autosteer_ESP32
extern float           P;                                 		//-- from AOG_Autosteer_ESP32
extern uint16_t        PWMOutFrequ;                       		//-- from AOG_Autosteer_ESP32
extern uint8_t         PWM_PIN;                           		//-- from AOG_Autosteer_ESP32
extern float           Pc;                                		//-- from AOG_Autosteer_ESP32
extern uint8_t         PressureSensor;                    		//-- from AOG_Autosteer_ESP32
extern byte            REPORT_INTERVAL;                   		//-- from AOG_Autosteer_ESP32
extern uint8_t         Relays_ON;                         		//-- from AOG_Autosteer_ESP32
extern uint8_t         SCL;                               		//-- from AOG_Autosteer_ESP32
extern uint8_t         SDA;                               		//-- from AOG_Autosteer_ESP32
extern uint8_t         STEERSW_PIN;                       		//-- from AOG_Autosteer_ESP32
extern byte            SentenceFromAOGLength;             		//-- from AOG_Autosteer_ESP32
extern byte            SentenceFromAOG[SentenceFromAOGMaxLength];		//-- from AOG_Autosteer_ESP32
extern uint8_t         Servo_PIN;                         		//-- from AOG_Autosteer_ESP32
extern uint8_t         ShaftEncoder;                      		//-- from AOG_Autosteer_ESP32
extern bool            SteerButtonPressed;                		//-- from AOG_Autosteer_ESP32
extern uint8_t         SteerSwitchType;                   		//-- from AOG_Autosteer_ESP32
extern uint8_t         UseMMA_X_Axis;                     		//-- from AOG_Autosteer_ESP32
extern uint8_t         WASType;                           		//-- from AOG_Autosteer_ESP32
extern uint8_t         WAS_Diff_GND_PIN;                  		//-- from AOG_Autosteer_ESP32
extern uint8_t         WAS_PIN;                           		//-- from AOG_Autosteer_ESP32
extern uint8_t         WORKSW_PIN;                        		//-- from AOG_Autosteer_ESP32
extern bool            WebIORunning;                      		//-- from AOG_Autosteer_ESP32
extern uint16_t        WebIOSteerPosZero;                 		//-- from AOG_Autosteer_ESP32
extern WiFiUDP         WiFiUDPFromAOG;                    		//-- from AOG_Autosteer_ESP32
extern bool            WiFiUDPRunning;                    		//-- from AOG_Autosteer_ESP32
extern WiFiUDP         WiFiUDPToAOG;                      		//-- from AOG_Autosteer_ESP32
extern byte            WiFi_STA_connect_call_nr;          		//-- from AOG_Autosteer_ESP32
extern WebServer       WiFi_Server;                       		//-- from AOG_Autosteer_ESP32
extern byte            WiFi_connect_step;                 		//-- from AOG_Autosteer_ESP32
extern byte            WiFi_ipDest_ending;                		//-- from AOG_Autosteer_ESP32
extern IPAddress       WiFi_ipDestination;                		//-- from AOG_Autosteer_ESP32
extern byte            WiFi_netw_nr;                      		//-- from AOG_Autosteer_ESP32
extern uint16_t        WorkSW_Threshold;                  		//-- from AOG_Autosteer_ESP32
extern uint8_t         WorkSW_mode;                       		//-- from AOG_Autosteer_ESP32
extern float           Xp;                                		//-- from AOG_Autosteer_ESP32
extern float           Zp;                                		//-- from AOG_Autosteer_ESP32
extern long            actualSteerPosRAW;                 		//-- from AOG_Autosteer_ESP32
extern ADS1115_lite    adc;                               		//-- from AOG_Autosteer_ESP32
extern uint8_t         aogVersion;                        		//-- from AOG_Autosteer_ESP32
extern long            argVal;                            		//-- from AOG_Autosteer_ESP32
extern float           autoSteerMaxSpeed;                 		//-- from AOG_Autosteer_ESP32
extern float           autoSteerMinSpeed;                 		//-- from AOG_Autosteer_ESP32
extern BNO080          bno08x;                            		//-- from AOG_Autosteer_ESP32
extern byte            bno08xAddress;                     		//-- from AOG_Autosteer_ESP32
extern float           bno08xHeading;                     		//-- from AOG_Autosteer_ESP32
extern int             bno08xHeading10x;                  		//-- from AOG_Autosteer_ESP32
extern double          bno08xPitch;                       		//-- from AOG_Autosteer_ESP32
extern double          bno08xRoll;                        		//-- from AOG_Autosteer_ESP32
extern int             bno08xRoll10x;                     		//-- from AOG_Autosteer_ESP32
extern bool            debugmode;                         		//-- from AOG_Autosteer_ESP32
extern bool            debugmodeDataFromAOG;              		//-- from AOG_Autosteer_ESP32
extern float           diff;                              		//-- from AOG_Autosteer_ESP32
extern float           distanceFromLine;                  		//-- from AOG_Autosteer_ESP32
extern uint8_t         encA_PIN;                          		//-- from AOG_Autosteer_ESP32
extern uint8_t         encB_PIN;                          		//-- from AOG_Autosteer_ESP32
extern bool            encDebounce;                       		//-- from AOG_Autosteer_ESP32
extern int             errorAbs;                          		//-- from AOG_Autosteer_ESP32
extern float           gpsSpeed;                          		//-- from AOG_Autosteer_ESP32
extern byte            guidanceStatus;                    		//-- from AOG_Autosteer_ESP32
extern float           heading;                           		//-- from AOG_Autosteer_ESP32
extern int16_t         heading16;                         		//-- from AOG_Autosteer_ESP32
extern int             highLowPerDeg;                     		//-- from AOG_Autosteer_ESP32
extern byte            highPWM;                           		//-- from AOG_Autosteer_ESP32
extern byte            incomSentenceDigit;                		//-- from AOG_Autosteer_ESP32
extern byte            incommingBytesArrayNr;             		//-- from AOG_Autosteer_ESP32
extern byte            incommingBytesArrayNrToParse;      		//-- from AOG_Autosteer_ESP32
extern byte            incommingBytes[incommingDataArraySize];		//-- from AOG_Autosteer_ESP32
extern bool            isSteerArdConfFound;               		//-- from AOG_Autosteer_ESP32
extern bool            isSteerArdConfFoundV17;            		//-- from AOG_Autosteer_ESP32
extern bool            isSteerArdConfigFound;             		//-- from AOG_Autosteer_ESP32
extern bool            isSteerArdConfigFoundV17;          		//-- from AOG_Autosteer_ESP32
extern bool            isSteerDataFound;                  		//-- from AOG_Autosteer_ESP32
extern bool            isSteerDataFoundV17;               		//-- from AOG_Autosteer_ESP32
extern bool            isSteerSettingFound;               		//-- from AOG_Autosteer_ESP32
extern bool            isSteerSettingFoundV17;            		//-- from AOG_Autosteer_ESP32
extern float           lastRoll;                          		//-- from AOG_Autosteer_ESP32
extern byte            lowPWM;                            		//-- from AOG_Autosteer_ESP32
extern byte            minPWM;                            		//-- from AOG_Autosteer_ESP32
extern byte            my_WiFi_Mode;                      		//-- from AOG_Autosteer_ESP32
extern bool            newDataFromAOG;                    		//-- from AOG_Autosteer_ESP32
extern int             nrBNO08xAdresses;                  		//-- from AOG_Autosteer_ESP32
extern uint8_t         output_type;                       		//-- from AOG_Autosteer_ESP32
extern float           pValue;                            		//-- from AOG_Autosteer_ESP32
extern int             prevEncAState;                     		//-- from AOG_Autosteer_ESP32
extern int             prevEncBState;                     		//-- from AOG_Autosteer_ESP32
extern int             pulseCount;                        		//-- from AOG_Autosteer_ESP32
extern uint8_t         pulseCountMax;                     		//-- from AOG_Autosteer_ESP32
extern int             pwmDisplay;                        		//-- from AOG_Autosteer_ESP32
extern int             pwmDrive;                          		//-- from AOG_Autosteer_ESP32
extern int             pwmOut;                            		//-- from AOG_Autosteer_ESP32
extern bool            remoteSwitchPressed;               		//-- from AOG_Autosteer_ESP32
extern float           roll;                              		//-- from AOG_Autosteer_ESP32
extern int16_t         roll16;                            		//-- from AOG_Autosteer_ESP32
extern float           rollMMA;                           		//-- from AOG_Autosteer_ESP32
extern uint16_t        roll_corr;                         		//-- from AOG_Autosteer_ESP32
extern float           steerAngleActual;                  		//-- from AOG_Autosteer_ESP32
extern float           steerAngleError;                   		//-- from AOG_Autosteer_ESP32
extern float           steerAngleSetPoint;                		//-- from AOG_Autosteer_ESP32
extern bool            steerEnable;                       		//-- from AOG_Autosteer_ESP32
extern bool            steerEnableOld;                    		//-- from AOG_Autosteer_ESP32
extern float           steerSensorCounts;                 		//-- from AOG_Autosteer_ESP32
extern byte            steerSwitch;                       		//-- from AOG_Autosteer_ESP32
extern long            steeringPosition;                  		//-- from AOG_Autosteer_ESP32
extern byte            switchByte;                        		//-- from AOG_Autosteer_ESP32
extern TaskHandle_t    taskHandle_DataFromAOGEth;         		//-- from AOG_Autosteer_ESP32
extern TaskHandle_t    taskHandle_DataFromAOGUSB;         		//-- from AOG_Autosteer_ESP32
extern TaskHandle_t    taskHandle_DataFromAOGWiFi;        		//-- from AOG_Autosteer_ESP32
extern TaskHandle_t    taskHandle_LEDBlink;               		//-- from AOG_Autosteer_ESP32
extern TaskHandle_t    taskHandle_WebIO;                  		//-- from AOG_Autosteer_ESP32
extern TaskHandle_t    taskHandle_WiFi_connect;           		//-- from AOG_Autosteer_ESP32
extern bool            toggleSteerEnable;                 		//-- from AOG_Autosteer_ESP32
extern const float     varProcess;                        		//-- from AOG_Autosteer_ESP32
extern const float     varRoll;                           		//-- from AOG_Autosteer_ESP32
extern byte            vers_nr;                           		//-- from AOG_Autosteer_ESP32
extern byte            watchdogTimer;                     		//-- from AOG_Autosteer_ESP32
extern byte            workSwitch;                        		//-- from AOG_Autosteer_ESP32
extern byte            workSwitchOld;                     		//-- from AOG_Autosteer_ESP32
extern uint16_t        x_;                                		//-- from AOG_Autosteer_ESP32
extern uint16_t        y_;                                		//-- from AOG_Autosteer_ESP32
extern uint16_t        z_;                                		//-- from AOG_Autosteer_ESP32
extern uint8_t         _cs;                               		//-- from BNO08x_AOG
extern uint8_t         _deviceAddress;                    		//-- from BNO08x_AOG
extern uint8_t         _int;                              		//-- from BNO08x_AOG
extern boolean         _printDebug;                       		//-- from BNO08x_AOG
extern uint8_t         _rst;                              		//-- from BNO08x_AOG
extern uint8_t         _wake;                             		//-- from BNO08x_AOG
extern uint16_t        accelAccuracy;                     		//-- from BNO08x_AOG
extern uint16_t        accelLinAccuracy;                  		//-- from BNO08x_AOG
extern int16_t         accelerometer_Q1;                  		//-- from BNO08x_AOG
extern uint8_t         activityClassifier;                		//-- from BNO08x_AOG
extern int16_t         angular_velocity_Q1;               		//-- from BNO08x_AOG
extern uint8_t         calibrationAccEnable;              		//-- from BNO08x_AOG
extern uint8_t         calibrationGyroEnable;             		//-- from BNO08x_AOG
extern uint8_t         calibrationMagnEnable;             		//-- from BNO08x_AOG
extern uint8_t         calibrationPlanEnable;             		//-- from BNO08x_AOG
extern uint8_t         calibrationStatus;                 		//-- from BNO08x_AOG
extern uint8_t         commandSequenceNumber;             		//-- from BNO08x_AOG
extern uint8_t         featureReportId;                   		//-- from BNO08x_AOG
extern uint16_t        gyroAccuracy;                      		//-- from BNO08x_AOG
extern int16_t         gyro_Q1;                           		//-- from BNO08x_AOG
extern int16_t         linear_accelerometer_Q1;           		//-- from BNO08x_AOG
extern uint16_t        magAccuracy;                       		//-- from BNO08x_AOG
extern int16_t         magnetometer_Q1;                   		//-- from BNO08x_AOG
extern uint16_t        memsRawAccelX;                     		//-- from BNO08x_AOG
extern uint16_t        memsRawAccelY;                     		//-- from BNO08x_AOG
extern uint16_t        memsRawAccelZ;                     		//-- from BNO08x_AOG
extern uint16_t        memsRawGyroX;                      		//-- from BNO08x_AOG
extern uint16_t        memsRawGyroY;                      		//-- from BNO08x_AOG
extern uint16_t        memsRawGyroZ;                      		//-- from BNO08x_AOG
extern uint16_t        memsRawMagX;                       		//-- from BNO08x_AOG
extern uint16_t        memsRawMagY;                       		//-- from BNO08x_AOG
extern uint16_t        memsRawMagZ;                       		//-- from BNO08x_AOG
extern uint32_t        metaData[MAX_METADATA_SIZE];       		//-- from BNO08x_AOG
extern uint16_t        quatAccuracy;                      		//-- from BNO08x_AOG
extern uint16_t        rawAccelX;                         		//-- from BNO08x_AOG
extern uint16_t        rawAccelY;                         		//-- from BNO08x_AOG
extern uint16_t        rawAccelZ;                         		//-- from BNO08x_AOG
extern uint16_t        rawFastGyroX;                      		//-- from BNO08x_AOG
extern uint16_t        rawFastGyroY;                      		//-- from BNO08x_AOG
extern uint16_t        rawFastGyroZ;                      		//-- from BNO08x_AOG
extern uint16_t        rawGyroX;                          		//-- from BNO08x_AOG
extern uint16_t        rawGyroY;                          		//-- from BNO08x_AOG
extern uint16_t        rawGyroZ;                          		//-- from BNO08x_AOG
extern uint16_t        rawLinAccelX;                      		//-- from BNO08x_AOG
extern uint16_t        rawLinAccelY;                      		//-- from BNO08x_AOG
extern uint16_t        rawLinAccelZ;                      		//-- from BNO08x_AOG
extern uint16_t        rawMagX;                           		//-- from BNO08x_AOG
extern uint16_t        rawMagY;                           		//-- from BNO08x_AOG
extern uint16_t        rawMagZ;                           		//-- from BNO08x_AOG
extern uint16_t        rawQuatI;                          		//-- from BNO08x_AOG
extern uint16_t        rawQuatJ;                          		//-- from BNO08x_AOG
extern uint16_t        rawQuatK;                          		//-- from BNO08x_AOG
extern uint16_t        rawQuatRadianAccuracy;             		//-- from BNO08x_AOG
extern uint16_t        rawQuatReal;                       		//-- from BNO08x_AOG
extern long            reportInterval;                    		//-- from BNO08x_AOG
extern int16_t         rotationVectorAccuracy_Q1;         		//-- from BNO08x_AOG
extern int16_t         rotationVector_Q1;                 		//-- from BNO08x_AOG
extern uint8_t         stabilityClassifier;               		//-- from BNO08x_AOG
extern uint16_t        stepCount;                         		//-- from BNO08x_AOG
extern uint32_t        timeStamp;                         		//-- from BNO08x_AOG
extern float           AOGSteerPositionZero;              		//-- from arduinoGlue
extern uint8_t         AckermanFix;                       		//-- from arduinoGlue
extern uint8_t         AutosteerLED_PIN;                  		//-- from arduinoGlue
extern float           BNOHeadingCorrection;              		//-- from arduinoGlue
extern float           BNORollCorrection;                 		//-- from arduinoGlue
extern uint8_t         BNOUsedAxis;                       		//-- from arduinoGlue
extern uint8_t         CAN_RX_PIN;                        		//-- from arduinoGlue
extern uint8_t         CAN_TX_PIN;                        		//-- from arduinoGlue
extern float           CMPS14HeadingCorrection;           		//-- from arduinoGlue
extern float           CMPS14RollCorrection;              		//-- from arduinoGlue
extern uint8_t         CMPS14UsedAxis;                    		//-- from arduinoGlue
extern int             CMPS14_ADDRESS;                    		//-- from arduinoGlue
extern uint8_t         CurrentSensor;                     		//-- from arduinoGlue
extern uint8_t         Current_sens_PIN;                  		//-- from arduinoGlue
extern uint8_t         DIR_PIN;                           		//-- from arduinoGlue
extern byte            DataTransVia;                      		//-- from arduinoGlue
extern uint8_t         Eth_CS_PIN;                        		//-- from arduinoGlue
extern byte            Eth_ipDest_ending;                 		//-- from arduinoGlue
extern bool            Eth_static_IP;                     		//-- from arduinoGlue
extern uint8_t         IMUType;                           		//-- from arduinoGlue
extern uint8_t         InvertRoll;                        		//-- from arduinoGlue
extern uint8_t         InvertWAS;                         		//-- from arduinoGlue
extern uint8_t         Invert_WorkSW;                     		//-- from arduinoGlue
extern float           Kd;                                		//-- from arduinoGlue
extern float           Ki;                                		//-- from arduinoGlue
extern float           Ko;                                		//-- from arduinoGlue
extern float           Kp;                                		//-- from arduinoGlue
extern uint8_t         LEDWiFi_ON_Level;                  		//-- from arduinoGlue
extern uint8_t         LEDWiFi_PIN;                       		//-- from arduinoGlue
extern uint8_t         MMAInstalled;                      		//-- from arduinoGlue
extern uint8_t         MMA_roll_MAX_STEP;                 		//-- from arduinoGlue
extern uint8_t         MotorDriveDirection;               		//-- from arduinoGlue
extern uint8_t         MotorSlowDriveDegrees;             		//-- from arduinoGlue
extern uint16_t        PWMOutFrequ;                       		//-- from arduinoGlue
extern uint8_t         PWM_PIN;                           		//-- from arduinoGlue
extern uint8_t         PressureSensor;                    		//-- from arduinoGlue
extern uint8_t         Relays_ON;                         		//-- from arduinoGlue
extern uint8_t         SCL;                               		//-- from arduinoGlue
extern uint8_t         SDA;                               		//-- from arduinoGlue
extern uint8_t         STEERSW_PIN;                       		//-- from arduinoGlue
extern uint8_t         Servo_PIN;                         		//-- from arduinoGlue
extern uint8_t         ShaftEncoder;                      		//-- from arduinoGlue
extern uint8_t         SteerSwitchType;                   		//-- from arduinoGlue
extern uint8_t         UseMMA_X_Axis;                     		//-- from arduinoGlue
extern uint8_t         WASType;                           		//-- from arduinoGlue
extern uint8_t         WAS_Diff_GND_PIN;                  		//-- from arduinoGlue
extern uint8_t         WAS_PIN;                           		//-- from arduinoGlue
extern uint8_t         WORKSW_PIN;                        		//-- from arduinoGlue
extern uint16_t        WebIOSteerPosZero;                 		//-- from arduinoGlue
extern byte            WiFi_ipDest_ending;                		//-- from arduinoGlue
extern uint16_t        WorkSW_Threshold;                  		//-- from arduinoGlue
extern uint8_t         WorkSW_mode;                       		//-- from arduinoGlue
extern uint8_t         aogVersion;                        		//-- from arduinoGlue
extern float           autoSteerMaxSpeed;                 		//-- from arduinoGlue
extern float           autoSteerMinSpeed;                 		//-- from arduinoGlue
extern bool            debugmode;                         		//-- from arduinoGlue
extern bool            debugmodeDataFromAOG;              		//-- from arduinoGlue
extern uint8_t         encA_PIN;                          		//-- from arduinoGlue
extern uint8_t         encB_PIN;                          		//-- from arduinoGlue
extern byte            highPWM;                           		//-- from arduinoGlue
extern byte            lowPWM;                            		//-- from arduinoGlue
extern byte            minPWM;                            		//-- from arduinoGlue
extern uint8_t         output_type;                       		//-- from arduinoGlue
extern uint8_t         pulseCountMax;                     		//-- from arduinoGlue
extern uint16_t        roll_corr;                         		//-- from arduinoGlue
extern float           steerSensorCounts;                 		//-- from arduinoGlue
extern byte            EE_ident2;                         		//-- from zAOG_ASt_EEPROM
extern char            HTML_String[40000];                		//-- from zAOG_ASt_WebInterface
extern bool            ping_start;                        		//-- from zAOG_ping

//============ Function Prototypes =========
//-- from zAGO_GPS_AssignGPIOs.ino -----------
void assignGPIOs_start_extHardware();                       
//-- from zAOG_ASt_Comm.ino -----------
void getDataFromAOGUSB(void* pvParameters);                 
void getDataFromAOGWiFi(void* pvParameters);                
void getDataFromAOGEth(void* pvParameters);                 
void parseDataFromAOG();                                    
void AOGDataSend();                                         
void SendTwoThirty(byte check);                             
//-- from zAOG_ASt_EEPROM.ino -----------
void restoreEEprom();                                       
byte EEprom_empty_check();                                  
void EEprom_write_all();                                    
void EEprom_read_all();                                     
void EEprom_read_default();                                 
void EEprom_block_restart();                                
void EEprom_unblock_restart();                              
void EEprom_show_memory();                                  
//-- from zAOG_ASt_Misc.ino -----------
void SetRelays(void);                                       
void WiFi_LED_blink(void* pvParameters);                    
//-- from zAOG_ASt_Network.ino -----------
void WiFi_handle_connection(void* pvParameters);            
void WiFi_scan_networks();                                  
void WiFi_STA_connect_network();                            
void WiFi_Start_AP();                                       
void Eth_handle_connection(void* pvParameters);             
//-- from zAOG_ASt_PWM_calc_and_output.ino -----------
void calcSteeringPID(void);                                 
void motorDrive(void);                                      
void motorDrive_Cytron(void);                               
void motorDrive_IBT_Mot(void);                              
void motorDrive_IBT_PWM(void);                              
void motorDrive_IBT_Danfoss(void);                          
//-- from zAOG_ASt_WebInterface.ino -----------
void doWebinterface(void* pvParameters);                    
void handleRoot();                                          
void WiFiStartServer();                                     
void process_Request();                                     
void make_HTML01();                                         
void handleNotFound();                                      
void set_colgroup(int w1, int w2, int w3, int w4, int w5);  
void set_colgroup(int w1, int w2, int w3, int w4, int w5, int w6);
void set_colgroup(int w1, int w2, int w3, int w4, int w5, int w6, int w7);
void set_colgroup1(int ww);                                 
void strcatf(char* tx, float f, byte leng, byte dezim);     
void strcati(char* tx, int i);                              

#endif // ARDUINOGLUE_H
