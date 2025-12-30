#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/mphal.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// Register address definitions
#define XDATA_REG 0x01
#define YDATA_REG 0x03
#define ZDATA_REG 0x05
#define STATUS_REG 0x09
#define CONTROL_1_REG 0x0A
#define CONTROL_2_REG 0x0B

// Constant definitions
#define M_PI 3.14159265358979323846
#define RAD_TO_DEG (180.0/M_PI)
#define DEFAULT_I2C_PORT_NUM -1
#define DEFAULT_I2C_ADDR 0
#define QMC5883P_I2C_ADDRESS 0x2C

#define CALIBRATION_TIMEOUT_MICROSEC 120e6
#define CALIBRATION_FIELD_RANGE_TOLERANCE 1.5f
#define CALIBRATION_RETURN_TOLERANCE 0.1f

// Object definition
typedef struct {
	mp_obj_base_t base;

	uint8_t i2c_address;
	i2c_master_bus_handle_t bus_handle;
	i2c_master_dev_handle_t device_handle;

	float data[3];
	float softcal[3];
	float hardcal[3];
} qmc5883p_obj_t;

// Struct used for returning caibration data
typedef struct {
    float xmaxmin[2];
    float ymaxmin[2];
    float zmaxmin[2];
} calibration_data;

// Function declarations
static void wait_micro_s(uint32_t micro_s_delay);
static void log_func(const char *log_string);
static void magnetometer_setup(qmc5883p_obj_t *self);
static void update_data(qmc5883p_obj_t *self);
static void mparray_to_float(mp_obj_t array, float* output);
static void normalize_vector(float* vector, float* output);
static void heading_vector(qmc5883p_obj_t* self, float* quaternion, float* output);
static void quat_rotate_mag_readings(qmc5883p_obj_t* self, float* quaternion, float* output);
static void calibrationrotation_data(qmc5883p_obj_t *self, float fieldstrength, calibration_data* output);
static uint8_t check_drdy(qmc5883p_obj_t *self);

extern const mp_obj_type_t qmc5883p_type;

#endif
