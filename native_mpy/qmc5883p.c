#include "qmc5883p.h"

mp_obj_t mpy_init(mp_obj_fun_bc_t *self, size_t n_args, size_t n_kw, mp_obj_t *args){
	MP_DYNRUNTIME_INIT_ENTRY;

    // Make functions dynamically available in python namespace here if needed

    MP_DYNRUNTIME_INIT_EXIT;

    return mp_const_none;
}

static mp_obj_t qmc5883p_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    nlr_buf_t cpu_state;
    mp_obj_t test_method[2];

	// Checking arguments
	mp_arg_check_num(n_args, n_kw, 1, 1, false);

    // Creating and allocating memory to the "self" instance of this module
	qmc5883p_obj_t *self = m_new_obj(qmc5883p_obj_t);

    if (nlr_push(&cpu_state) == 0){
        mp_load_method(args[0], MP_QSTR_writeto_mem, test_method);
        mp_load_method(args[0], MP_QSTR_readfrom_mem, test_method);

        nlr_pop();
    }
    else {
        mp_raise_ValueError("I2C bus object not valid");
    }

    self->address = 0x2C;
    self->i2c_bus = args[0];

    self->data[0] = self->data[1] = self->data[2] = 0.0f;
    self->softcal[0] = self->softcal[1] = self->softcal[2] = 1.0f;
    self->hardcal[0] = self->hardcal[1] = self->hardcal[2] = 0.0f;

    mp_hal_delay_us(250);

    magnetometer_setup(self);

    return MP_OBJ_FROM_PTR(self);
}

static void magnetometer_setup(qmc5883p_obj_t *self){
    nlr_buf_t cpu_state;

	// Loading writeto_mem method
    mp_obj_t writeto_method[2];
    mp_load_method(self->i2c_bus, MP_QSTR_writeto_mem, writeto_method);

    // Creating arguments lists
    // Module in normal power mode, 200Hz output rate, oversampling=4, downsampling=0
    mp_obj_t control1_data[3] = {mp_obj_new_int(self->address), mp_obj_new_int(CONTROL_1_REG), mp_obj_new_bytes((const byte[]){0x1D}, 1)};
    mp_obj_t control2_data[3] = {mp_obj_new_int(self->address), mp_obj_new_int(CONTROL_2_REG), mp_obj_new_bytes((const byte[]){0x0C}, 1)};

    if (nlr_push(&cpu_state) == 0){
        // Writing values
        mp_call_function_n_kw(writeto_method[0], 3, 0, control1_data);
        mp_call_function_n_kw(writeto_method[0], 3, 0, control2_data);

        nlr_pop();
    }
    else {
        mp_raise_OSError(MP_ENODEV);
    }
}

static uint8_t* bytearray_to_array(mp_obj_t bytearray){
    // Utility to get the pointer from a mp_obj_t bytearray so the data can be accessed
    mp_buffer_info_t buf_info;

    mp_get_buf_raise(bytearray, &buf_info, MP_BUFFER_READ);

    const uint8_t *data = (const uint8_t *)buf_info.buf;

    return data;
}

static float* mp_array_to_c_array(mp_obj_t array){
    size_t len;
    mp_obj_t *items;
    int i;
    float *buffer = malloc(4*sizeof(float));

    if (buffer == NULL){
        // Error: out of memory
        mp_raise_OSError(MP_ENOMEM);
    }

    mp_obj_get_array(array, &len, &items);

    if (len != 4){
        mp_raise_ValueError("Expected 4 values in list/tuple");
    }

    for (i=0; i < 4; i++){
        buffer[i] = mp_obj_get_float(items[i]);
    }

    return buffer;
}

static float* normalize_vector(float *vector){
    float sum_sq, sum;
    float *vector_normalized = malloc(3*sizeof(float));

    sum_sq = vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2];
    sum = sqrt(sum_sq);

    // Guarding against 0-division error
    if (sum < 1e-10f){
        free(vector);
        mp_raise_ValueError("Magnetometer reading too small (vector length < 1e-10 Gauss). Can't normalize vector.");
    }
    else{
        vector_normalized[0] = vector[0] / sum;
        vector_normalized[1] = vector[1] / sum;
        vector_normalized[2] = vector[2] / sum;
    }

    return vector_normalized;
}

static float* quat_rotate_mag_readings(qmc5883p_obj_t *self, float *quaternion){
    float qw, qx, qy, qz;
    float *world_magnetometer = malloc(2*sizeof(float));
    float *normalized_mag;

    if (world_magnetometer == NULL){
        // Error: out of memory
        mp_raise_OSError(MP_ENOMEM);
    }

    qw = quaternion[0];
    qx = quaternion[1];
    qy = quaternion[2];
    qz = quaternion[3];

    normalized_mag = normalize_vector(self->data);

    // Using quaternion rotation to find the magnetic north direction vector in the world reference frame
    world_magnetometer[0] = (qw*qw + qx*qx - qy*qy - qz*qz)*normalized_mag[0] + 2*(qx*qy - qw*qz)*normalized_mag[1] + 2*(qx*qz + qw*qy)*normalized_mag[2];
    world_magnetometer[1] = 2*(qx*qy + qw*qz)*normalized_mag[0] + (qw*qw - qx*qx + qy*qy - qz*qz)*normalized_mag[1] + 2*(qy*qz - qw*qx)*normalized_mag[2];

    free(normalized_mag);

    return world_magnetometer;
}

static float* heading_vector(qmc5883p_obj_t *self, float *quaternion){
    float qw, qx, qy, qz;
    float *world_heading = malloc(2*sizeof(float));

    if (world_heading == NULL){
        // Error: out of memory
        mp_raise_OSError(MP_ENOMEM);
    }

    qw = quaternion[0];
    qx = quaternion[1];
    qy = quaternion[2];
    qz = quaternion[3];

    // Using quaternion rotation to find the magnetometer's heading vector in the world reference frame
    world_heading[0] = (qw*qw + qx*qx - qy*qy - qz*qz)*-1.0f;
    world_heading[1] = (qx*qy + qw*qz)*-2.0f;

    return world_heading;
}

static int check_drdy(qmc5883p_obj_t *self){
    mp_obj_t readfrom_method[2];
    mp_obj_t statusreg_data;
    uint8_t *statusreg_intdata;
    int counter = 0;
    nlr_buf_t cpu_state;

    mp_load_method(self->i2c_bus, MP_QSTR_readfrom_mem, readfrom_method);
    mp_obj_t arguments[3] = {mp_obj_new_int(self->address), mp_obj_new_int(STATUS_REG), mp_obj_new_int(1)};

    // Checking the DRDY bit of status register
    while (counter < 2){
        // Making sure that the loop times out after two attempts
        counter ++;

        // Reading from the sensor, then checking the bit

        if (nlr_push(&cpu_state) == 0){
            statusreg_data = mp_call_function_n_kw(readfrom_method[0], 3, 0, arguments);

            nlr_pop();
        }
        else {
            mp_raise_OSError(MP_ENODEV);
        }

        statusreg_intdata = bytearray_to_array(statusreg_data);

        if (statusreg_intdata[0] & 0x01){
            return 1;
        }
        else {
            mp_hal_delay_ms(5);
        }
    }

    return 0;
}

static int update_data(qmc5883p_obj_t *self){
    mp_obj_t mag_data;
    mp_obj_t readfrom_method[2];
    uint8_t *mag_intdata;
    float xdata, ydata, zdata;
    int flag;
    nlr_buf_t cpu_state;

    mp_load_method(self->i2c_bus, MP_QSTR_readfrom_mem, readfrom_method);

    // Checking if there's new data available
    flag = check_drdy(self);
    if (flag == 0){
        return 0;
    }

    // Burst reading the 6 data bytes from the sensor
    mp_obj_t data_arguments[3] = {mp_obj_new_int(self->address), mp_obj_new_int(XDATA_REG), mp_obj_new_int(6)};

    if (nlr_push(&cpu_state) == 0){
        mag_data = mp_call_function_n_kw(readfrom_method[0], 3, 0, data_arguments);
        nlr_pop();
    }
    else {
        mp_raise_OSError(MP_ENODEV);
    }

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

    return 1;
}

static mp_obj_t getdata_raw(mp_obj_t self_in){
    int flag;
    mp_obj_t data_array[3];

    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);

    flag = update_data(self);

    // Creating an array of python floats to return
    data_array[0] = mp_obj_new_float(self->data[0]);
    data_array[1] = mp_obj_new_float(self->data[1]);
    data_array[2] = mp_obj_new_float(self->data[2]);

    return mp_obj_new_list(3, data_array);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(qmc5883p_getdata_raw_obj, getdata_raw);

static mp_obj_t compass_2d(mp_obj_t self_in, mp_obj_t dec){
    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);
    float declination = mp_obj_get_float(dec);
    float heading;
    uint16_t heading_rounded;
    int flag;

    flag = update_data(self);

    // Calculating heading
    heading = (atan2f(self->data[1], -self->data[0]) * RAD_TO_DEG) - declination;

    // Ensuring heading goes from 0 -> 360 degrees
    heading %= 360;

    // Rounding heading to nearest degree
    heading_rounded = (uint16_t)(heading+0.5f);

    return mp_obj_new_int_from_uint(heading_rounded);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(qmc5883p_compass_2d_obj, compass_2d);

static mp_obj_t compass_3d(mp_obj_t self_in, mp_obj_t quaternion, mp_obj_t dec){
    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);
    float declination = mp_obj_get_float(dec);
    float *quat = NULL, *headingvector = NULL, *mag_directionvector = NULL;
    float dotproduct, crossproduct_z, heading;
    uint16_t heading_rounded;
    int flag;
    nlr_buf_t cpu_state;

    // Error handling 
    // If any of these functions fail, the required memory will be tidied up and then the error code returned
    if (nlr_push(&cpu_state) == 0){
        quat = mp_array_to_c_array(quaternion);

        flag = update_data(self);

        mag_directionvector = quat_rotate_mag_readings(self, quat);
        headingvector = heading_vector(self, quat);

        nlr_pop();
    }
    else {
        if (quat){
            free(quat);
        }
        if (headingvector){
            free(headingvector);
        }
        if (mag_directionvector){
            free(mag_directionvector);
        }

        nlr_jump(cpu_state.ret_val);
    }

    // Heading calc maths: cross product = |a|*|b|*sin(theta), dot product = |a|*|b|*cos(theta)
    // So atan(crossproduct/dotproduct)=atan(sin(theta)/cos(theta))=atan(tan(theta))=theta
    // I.e. this code finds the angle between the magnetic north direction vector, and the module's heading_vector
    dotproduct = mag_directionvector[0]*headingvector[0] + mag_directionvector[1]*headingvector[1];
    crossproduct_z = mag_directionvector[0]*headingvector[1] - mag_directionvector[1]*headingvector[0];

    heading = (atan2f(crossproduct_z, dotproduct) * RAD_TO_DEG) - declination;

    // Ensuring heading goes from 0->360 degrees
    heading %= 360;
    heading_rounded = (uint16_t)(heading+0.5f);

    free(quat);
    free(headingvector);
    free(mag_directionvector);

    return mp_obj_new_int_from_uint(heading_rounded);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(qmc5883p_compass_3d_obj, compass_3d);


// TODO: FIGURE OUT HOW TO DO LOGGING
// TODO: ADD CALIBRATION FUNCTION



// Defining the functions that are exposed to micropython
STATIC const mp_rom_map_elem_t qmc5883p_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_getdata_raw), MP_ROM_PTR(&qmc5883p_getdata_raw_obj)},
    {MP_ROM_QSTR(MP_QSTR_compass_2d), MP_ROM_PTR(&qmc5883p_compass_2d_obj)},
    {MP_ROM_QSTR(MP_QSTR_compass_3d), MP_ROM_PTR(&qmc5883p_compass_3d_obj)},
};
STATIC MP_DEFINE_CONST_DICT(qmc5883p_locals_dict, qmc5883p_locals_dict_table);

// Overall module definition
const mp_obj_type_t qmc5883p = {
    .base = {&mp_type_module},
    .name = MP_QSTR_QMC5883P,
    .make_new = qmc5883p_make_new,
    .locals_dict = (mp_obj_dict_t *)&qmc5883p_locals_dict,
};