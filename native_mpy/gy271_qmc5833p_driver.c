#include "py/dynruntime.h"
#include "py/mphal.h"
#include "gy271_qmc5833p_driver.h"

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args){
	MP_DYNRUNTIME_INIT_ENTRY;

    // Making functions available in python namespace

    MP_DYNRUNTIME_INIT_EXIT;
    return mp_const_none;
}

static mp_obj_t qmc5883p_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
	// Checking arguments
	mp_arg_check_num(n_args, n_kw, 1, 1, false);

    // Creating and allocating memory to the "self" instance of this module
	qmc5883p_obj_t *self = m_new_obj(qmc5883p_obj_t);

    self->address = 0x2C;
    self->i2c_bus = args[0];

    self->data[0] = self->data[1] = self->data[2] = 0.0f;
    self->softcal[0] = self->softcal[1] = self->softcal[2] = 1.0f;
    self->hardcal[0] = self->hardcal[1] = self->hardcal[2] = 0.0f;

    mp_hal_delay_us(250);

    magnetometer_setup(*self);

    return MP_OBJ_FROM_PTR(self);
}

static void magnetometer_setup(qmc5883p_obj_t self){
	// Loading writeto_mem method
    mp_obj_t writeto_method[2];
    mp_load_method(self->i2c_bus, MP_QSTR_writeto_mem, writeto_method);

    // Creating arguments lists
    // Module in normal power mode, 200Hz output rate, oversampling=4, downsampling=0
    mp_obj_t control1_data[3] = {mp_obj_new_int(self->address), mp_obj_new_int(CONTROL_1_REG), mp_obj_new_bytes((const byte[]){0x1D}, 1)};
    mp_obj_t control2_data[3] = {mp_obj_new_int(self->address), mp_obj_new_int(CONTROL_2_REG), mp_obj_new_bytes((const byte[]){0x0C}, 1)};

    // Writing values
    mp_call_function_n_kw(writeto_method[0], 3, 0, control1_data);
    mp_call_function_n_kw(writeto_method[0], 3, 0, control2_data);
}

static uint8_t *bytearray_to_array(mp_obj_t bytearray){
    int i;

    // Utility to convert from a mp_obj_t bytearray to C array type
    mp_buffer_info_t buf_info;

    mp_get_buf_raise(bytearray, &buf_info, MP_BUFFER_READ);

    const uint8_t *data = (const uint8_t *)buf_info.buf;
    size_t length = buf_info.len;

    uint8_t *buffer = malloc(length);

    if (buffer == NULL){
        mp_raise_OSError(MP_ENOMEM);
    }

    for (i=0, i < length, i++){
        buffer[i] = data[i];
    }

    return buffer;
}

static int update_data(qmc5883p_obj_t *self){
    mp_obj_t statusreg_data;
    mp_obj_t mag_data;
    uint8_t statusreg_int;
    uint8_t *statusreg_intdata;
    uint8_t *mag_intdata;

    float xdata, ydata, zdata;

    mp_obj_t readfrom_method[2];
    int counter = 0;
    int drdy = 0;

    mp_load_method(self->i2c_bus, MP_QSTR_readfrom_mem, readfrom_method);
    mp_obj_t arguments[3] = {mp_obj_new_int(self->address), mp_obj_new_int(STATUS_REG), mp_obj_new_int(1)};

    // Checking the DRDY bit of status register
    while (drdy != 1){
        mp_hal_delay_us(5);

        // Making sure that the loop times out after two attempts
        counter ++;

        if (counter > 2){
            return 0;
        }

        // Reading from the sensor, then checking the bit
        statusreg_data = mp_call_function_n_kw(readfrom_method[0], 3, 0, arguments);
        statusreg_intdata = bytearray_to_array(statusreg_data);
        statusreg_int = statusreg_intdata[0];

        free(statusreg_intdata);

        if (statusreg_int & 0x01){
            drdy = 1;
        }
    }

    // Burst reading the 6 data bytes from the sensor
    mp_obj_t data_arguments[3] = {mp_obj_new_int(self->address), mp_obj_new_int(XDATA_REG), mp_obj_new_int(6)};
    mag_data = mp_call_function_n_kw(readfrom_method[0], 3, 0, data_arguments);

    mag_intdata = bytearray_to_array(mag_data);

    // Decoding the data into floats
    xdata = (int16_t)((mag_intdata[1] << 8) | mag_intdata[0]) / 15000.0f;
    ydata = (int16_t)((mag_intdata[3] << 8) | mag_intdata[2]) / 15000.0f;
    zdata = (int16_t)((mag_intdata[5] << 8) | mag_intdata[4]) / 15000.0f;

    xdata = (xdata - self->hardcal[0]) * self->softcal[0];
    ydata = (ydata - self->hardcal[1]) * self->softcal[1];
    zdata = (zdata - self->hardcal[2]) * self->softcal[2];

    self->data[0] = xdata;
    self->data[1] = ydata;
    self->data[2] = zdata;

    free(mag_intdata);

    return 1;
}