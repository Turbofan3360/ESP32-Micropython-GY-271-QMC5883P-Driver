#include <stdint.h>
#include <math.h>

#include "py/dynruntime.h"
#include "py/mphal.h"
#include "py/nlr.h"

// Register address definitions
#define CHIPID_REG 0x00
#define XDATA_REG 0x01
#define YDATA_REG 0x03
#define ZDATA_REG 0x05
#define AXIS_INVERT_REG 0x29
#define STATUS_REG 0x09
#define CONTROL_1_REG 0x0A
#define CONTROL_2_REG 0x0B

// Constant definitions
#define M_PI 3.14159265358979323846
#define RAD_TO_DEG (180/M_PI)

// Function declarations
static void magnetometer_setup(qmc5883p_obj_t *self);
static uint8_t* bytearray_to_array(mp_obj_t bytearray);
static float* mp_array_to_c_array(mp_obj_t array);
static float* normalize_vector(float *vector);
static float* quat_rotate_mag_readings(qmc5883p_obj_t *self, float *quaternion);
static float* heading_vector(qmc5883p_obj_t *self, float *quaternion);
static int check_drdy(qmc5883p_obj_t *self);
static int update_data(qmc5883p_obj_t *self);


typedef struct _qmc5883p_obj_t {
	mp_obj_base_t base;
	mp_obj_t i2c_bus;
	uint8_t address;
	float data[3];
	float softcal[3];
	float hardcal[3];
} qmc5883p_obj_t;