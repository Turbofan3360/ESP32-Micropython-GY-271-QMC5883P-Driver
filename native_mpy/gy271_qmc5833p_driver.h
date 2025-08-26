#include <stdint.h>
#include <math.h>

#include "py/dynruntime.h"
#include "py/mphal.h"

// Register address definitions
#define CHIPID_REG 0x00;
#define XDATA_REG 0x01;
#define YDATA_REG 0x03;
#define ZDATA_REG 0x05;
#define AXIS_INVERT_REG 0x29;
#define STATUS_REG 0x09;
#define CONTROL_1_REG 0x0A;
#define CONTROL_2_REG 0x0B;

// Constant definitions
#define M_PI 3.14159265358979323846
#define RAD_TO_DEG 180/M_PI;

typedef struct _qmc5883p_obj_t {
	mp_obj_base_t base;
	mp_obj_t i2c_bus;
	uint8_t address;
	float data[3];
	float softcal[3];
	float hardcal[3];
} qmc5883p_obj_t;