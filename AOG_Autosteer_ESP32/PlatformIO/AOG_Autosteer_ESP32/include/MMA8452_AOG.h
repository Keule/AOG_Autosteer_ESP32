#ifndef MMA8452_AOG_H_
#define MMA8452_AOG_H_
#ifndef MMA8452_H_

//============ Includes ====================
#include "arduinoGlue.h"

	//-- moved to arduinoGlue.h // #define MMA8452_H_

#ifdef ARDUINO
//#include <Arduino.h>                              		//-- moved to arduinoGlue.h
//#include <Wire.h>                                 		//-- moved to arduinoGlue.h
#else
//#include <stdlib.h>                               		//-- moved to arduinoGlue.h
//#include <avr/io.h>                               		//-- moved to arduinoGlue.h
#include "i2cmaster.h"

//============ Added by Convertor ==========

//#include <util/delay.h>                           		//-- moved to arduinoGlue.h
#endif

// I2C address set in hardware (tied high or low)

	//-- moved to arduinoGlue.h // #define MMA_ADDRESS 0x1C

//Register Map
	//-- moved to arduinoGlue.h // #define STATUS        0x00  // Real time status
	//-- moved to arduinoGlue.h // #define OUT_X_MSB     0x01  // [7:0] are 8 MSBs of 12-bit sample
	//-- moved to arduinoGlue.h // #define OUT_X_LSB     0x02  // [7:4] are 4 LSBs of 12-bit sample
	//-- moved to arduinoGlue.h // #define OUT_Y_MSB     0x03  // [7:0] are 8 MSBs of 12-bit sample
	//-- moved to arduinoGlue.h // #define OUT_Y_LSB     0x04  // [7:4] are 4 LSBs of 12-bit sample
	//-- moved to arduinoGlue.h // #define OUT_Z_MSB     0x05  // [7:0] are 8 MSBs of 12-bit sample
	//-- moved to arduinoGlue.h // #define OUT_Z_LSB     0x06  // [7:4] are 4 LSBs of 12-bit sample
// Reserved 0x07-0x08
	//-- moved to arduinoGlue.h // #define FIFO_SETUP    0x09  // FIFO Setup (8451 only)
	//-- moved to arduinoGlue.h // #define SYSMOD        0x0B  // Current System Mode
	//-- moved to arduinoGlue.h // #define INT_SOURCE      0x0C  // Interrupt status
	//-- moved to arduinoGlue.h // #define WHO_AM_I      0x0D  // Device ID (0x2A)
	//-- moved to arduinoGlue.h // #define XYZ_DATA_CFG    0x0E  // HPF Data Out and Dynamic Range Settings
	//-- moved to arduinoGlue.h // #define HP_FILTER_CUTOFF  0x0F  // Cutoff frequency is set to 16 Hz @ 800 Hz
	//-- moved to arduinoGlue.h // #define PL_STATUS     0x10  // Landscape/Portrait orientation status
	//-- moved to arduinoGlue.h // #define PL_CFG        0x11  // Landscape/Portrait configuration
	//-- moved to arduinoGlue.h // #define PL_COUNT      0x12  // Landscape/Portrait debounce counter
	//-- moved to arduinoGlue.h // #define PL_BF_ZCOMP     0x13  // Back-Front, Z-Lock Trip threshold
	//-- moved to arduinoGlue.h // #define P_L_THS_REG     0x14  // Portrait to Landscape Trip Angle is 29°
	//-- moved to arduinoGlue.h // #define FF_MT_CFG     0x15  // Freefall/Motion functional block configuration
	//-- moved to arduinoGlue.h // #define FF_MT_SRC     0x16  // Freefall/Motion event source register
	//-- moved to arduinoGlue.h // #define FF_MT_THS     0x17  // Freefall/Motion threshold register
	//-- moved to arduinoGlue.h // #define FF_MT_COUNT     0x18  // Freefall/Motion debounce counter
// Reserved 0x19-0x1C
	//-- moved to arduinoGlue.h // #define TRANSIENT_CFG   0x1D  // Transient functional block configuration
	//-- moved to arduinoGlue.h // #define TRANSIENT_SRC   0x1E  // Transient event status register
	//-- moved to arduinoGlue.h // #define TRANSIENT_THS   0x1F  // Transient event threshold
	//-- moved to arduinoGlue.h // #define TRANSIENT_COUNT   0x20  // Transient debounce counter
	//-- moved to arduinoGlue.h // #define PULSE_CFG     0x21  // ELE, Double_XYZ or Single_XYZ
	//-- moved to arduinoGlue.h // #define PULSE_SRC     0x22  // EA, Double_XYZ or Single_XYZ
	//-- moved to arduinoGlue.h // #define PULSE_THSX      0x23  // X pulse threshold
	//-- moved to arduinoGlue.h // #define PULSE_THSY      0x24  // Y pulse threshold
	//-- moved to arduinoGlue.h // #define PULSE_THSZ      0x25  // Z pulse threshold
	//-- moved to arduinoGlue.h // #define PULSE_TMLT      0x26  // Time limit for pulse
	//-- moved to arduinoGlue.h // #define PULSE_LTCY      0x27  // Latency time for 2nd pulse
	//-- moved to arduinoGlue.h // #define PULSE_WIND      0x28  // Window time for 2nd pulse
	//-- moved to arduinoGlue.h // #define ASLP_COUNT      0x29  // Counter setting for Auto-SLEEP
	//-- moved to arduinoGlue.h // #define CTRL_REG1     0x2A  // Data Rate, ACTIVE Mode
	//-- moved to arduinoGlue.h // #define CTRL_REG2     0x2B  // Sleep Enable, OS Modes, RST, ST
	//-- moved to arduinoGlue.h // #define CTRL_REG3     0x2C  // Wake from Sleep, IPOL, PP_OD
	//-- moved to arduinoGlue.h // #define CTRL_REG4     0x2D  // Interrupt enable register
	//-- moved to arduinoGlue.h // #define CTRL_REG5     0x2E  // Interrupt pin (INT1/INT2) map
	//-- moved to arduinoGlue.h // #define OFF_X       0x2F  // X-axis offset adjust
	//-- moved to arduinoGlue.h // #define OFF_Y       0x30  // Y-axis offset adjust
	//-- moved to arduinoGlue.h // #define OFF_Z       0x31  // Z-axis offset adjust
// Reserved 0x40-0x7F

/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_RANGE_2G = 0,
	MMA_RANGE_4G,
	MMA_RANGE_8G
} mma8452_range_t;
*/

/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_STANDBY = 0,
	MMA_WAKE,
	MMA_SLEEP
} mma8452_mode_t;
*/

// See table on page 23 in datasheet (http://www.freescale.com/files/sensors/doc/data_sheet/MMA8452Q.pdf)
/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_HP1 = 0,
	MMA_HP2,
	MMA_HP3,
	MMA_HP4
} mma8452_highpass_mode_t;
*/

/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_PORTRAIT_UP = 0,
	MMA_PORTRAIT_DOWN,
	MMA_LANDSCAPE_RIGHT,
	MMA_LANDSCAPE_LEFT
} mma8452_orientation_t;
*/

/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_FREEFALL = 0,
	MMA_MOTION
} mma8452_motion_type_t;
*/

// sleep sampling mode
/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_SLEEP_50hz = 0,
	MMA_SLEEP_12_5hz,
	MMA_SLEEP_6_25hz,
	MMA_SLEEP_1_56hz
} mma8452_sleep_frequency_t;
*/

// normal running mode
/*				*** enum moved to arduinoGlue.h ***
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
*/

// power mode
/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_NORMAL = 0,
	MMA_LOW_NOISE_LOW_POWER,
	MMA_HIGH_RESOLUTION,
	MMA_LOW_POWER
} mma_power_mode_t;
*/

/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_AUTO_SLEEP = 0x80,
	MMA_TRANSIENT = 0x20,
	MMA_ORIENTATION_CHANGE = 0x10,
	MMA_TAP = 0x08,
	MMA_FREEFALL_MOTION = 0x04,
	MMA_DATA_READY = 0x01
} mma_interrupt_types_t;
*/

/*				*** enum moved to arduinoGlue.h ***
typedef enum {
	MMA_X = 0x01,
	MMA_Y = 0x02,
	MMA_Z = 0x04,
	MMA_ALL_AXIS = 0x07
} mma_axis_t;
*/

class MMA8452
{
	public:
    MMA8452(uint8_t i2cAddress = MMA_ADDRESS);
		bool init();

		void setRange(mma8452_range_t range);
		mma8452_range_t getRange();

		void getRawData(uint16_t *x, uint16_t *y, uint16_t *z);
		void getAcceleration(float *x, float *y, float *z);

		mma8452_mode_t getMode();

		// todo: implement Pulse_LPF_EN
		void setHighPassFilter(bool enabled, mma8452_highpass_mode_t mode = MMA_HP1);

		void setAutoSleep(bool enabled, uint8_t time, mma8452_sleep_frequency_t sleepFrequencySampling = MMA_SLEEP_1_56hz, mma_power_mode_t sleepPowerMode = MMA_LOW_POWER);

		void setDataRate(mma_datarate_t dataRate);
		void setLowNoiseMode(bool enabled);
		void set8BitMode(bool enabled);

		void reset(); // todo: might not be working

		void setPowerMode(mma_power_mode_t powerMode);

		// 2mG/LSB
		void setOffsets(int8_t x, int8_t y, int8_t z);

    void setActive(bool active = true);

	private:
		bool active;

		mma8452_range_t range;

		void standby(bool standby);
		uint8_t read(uint8_t reg);
    bool readMultiple(uint8_t reg, uint8_t *buffer, uint8_t numuint8_ts);
		void write(uint8_t reg, uint8_t value);

		bool singleTapEnabled;
		bool doubleTapEnabled;
    uint8_t MMA8452_ADDRESS;

		float convertGCounts(uint16_t data);
		int8_t convertTo2sComplement(int8_t value);
};

#endif // MMA8452_H_
#endif // MMA8452_AOG_H_