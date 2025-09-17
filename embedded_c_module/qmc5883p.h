#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
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
#define RAD_TO_DEG (180.0/M_PI)
#define FLOAT_SIZE sizeof(float)

// Object definition
typedef struct {
	mp_obj_base_t base;
	mp_obj_t i2c_bus;
	uint8_t address;
	float data[3];
	float softcal[3];
	float hardcal[3];
} qmc5883p_obj_t;

// Struct used for returning caibration data
typedef struct {
	float *xdata;
	uint16_t xlength;
	float *ydata;
	uint16_t ylength;
	float *zdata;
	uint16_t zlength;
} calibration_data;

// Function declarations
static void log_func(const char *log_string);
static void magnetometer_setup(qmc5883p_obj_t *self);
static void update_data(qmc5883p_obj_t *self);
static float* mparray_to_float(mp_obj_t array);
static float* normalize_vector(float *vector);
static float* quat_rotate_mag_readings(qmc5883p_obj_t *self, float *quaternion);
static float* heading_vector(qmc5883p_obj_t *self, float *quaternion);
static float* max_min_average_array(float* array, uint16_t length, uint8_t num_to_average);
static float list_values_range(float *list, uint16_t length);
static const uint8_t* mparray_to_int(mp_obj_t bytearray);
static uint8_t is_in_array(float* array, uint16_t length, float item);
static uint8_t check_drdy(qmc5883p_obj_t *self);
static calibration_data* calibrationrotation_data(qmc5883p_obj_t *self, float fieldstrength);

extern const mp_obj_type_t qmc5883p_type;

#endif