#include "qmc5883p.h"

mp_obj_t qmc5883p_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    /**
     * This function initialises a new driver instance. It checks the I2C bus object is valid, and then initialises the module object.
     * It then calls magnetometer_setup to configure the QMC5883P's config registers as required.
    */
    nlr_buf_t cpu_state;
    mp_obj_t test_method[2];

	// Checking arguments
	mp_arg_check_num(n_args, n_kw, 1, 1, false);

    // Creating and allocating memory to the "self" instance of this module
	qmc5883p_obj_t *self = m_new_obj(qmc5883p_obj_t);

    if (nlr_push(&cpu_state) == 0){
        // Testing to see if the required I2C methods can be loaded from the I2C object
        mp_load_method(args[0], MP_QSTR_writeto_mem, test_method);
        mp_load_method(args[0], MP_QSTR_readfrom_mem, test_method);

        nlr_pop();
    }
    else {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("I2C bus object not valid"));
    }

    // Initialising the required data in the "self" object
    self->base.type = &qmc5883p_type;
    self->address = 0x2C;
    self->i2c_bus = args[0];

    self->data[0] = self->data[1] = self->data[2] = 0.0f;
    self->softcal[0] = self->softcal[1] = self->softcal[2] = 1.0f;
    self->hardcal[0] = self->hardcal[1] = self->hardcal[2] = 0.0f;

    mp_hal_delay_ms(1);

    magnetometer_setup(self);

    return MP_OBJ_FROM_PTR(self);
}

static void magnetometer_setup(qmc5883p_obj_t *self){
    /**
     * This function configures the QMC5883P chip's registers to the required settings (described below)
    */
    nlr_buf_t cpu_state;

	// Loading writeto_mem method
    mp_obj_t writeto_mem_method[2];
    mp_load_method(self->i2c_bus, MP_QSTR_writeto_mem, writeto_mem_method);

    // Creating arguments arrays
    // Module in normal power mode, 200Hz output rate, oversampling=4, downsampling=0
    mp_obj_t control1_data[5] = {writeto_mem_method[0], writeto_mem_method[1], mp_obj_new_int(self->address), mp_obj_new_int(CONTROL_1_REG), mp_obj_new_bytes((const byte[]){0x1D}, 1)};
    mp_obj_t control2_data[5] = {writeto_mem_method[0], writeto_mem_method[1], mp_obj_new_int(self->address), mp_obj_new_int(CONTROL_2_REG), mp_obj_new_bytes((const byte[]){0x0C}, 1)};

    if (nlr_push(&cpu_state) == 0){
        // Writing values
        mp_call_method_n_kw(3, 0, control1_data);
        mp_hal_delay_ms(5);
        mp_call_method_n_kw(3, 0, control2_data);

        nlr_pop();
    }
    else {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("Magnetometer setup failed - could not communicate with module."));
    }
}

static void log_func(const char *log_string){
    /**
     * Basic logging function - currently just prints to the REPL, but can be adapted to log to other places (e.g. log to a file) if needed
    */
    mp_printf(&mp_plat_print, "%s", log_string);
}

static const uint8_t* mparray_to_int(mp_obj_t bytearray){
    /**
     * Converts from a micropython bytearray to array of C ints
     * I.e. converts from python bytearray to a C array type
    */
    mp_buffer_info_t buf_info;

    mp_get_buffer_raise(bytearray, &buf_info, MP_BUFFER_READ);

    const uint8_t *data = (const uint8_t *)buf_info.buf;

    return data;
}

static void mparray_to_float(mp_obj_t array, float* output){
    /**
     * Converts from micropython array to an array of C floats
     * Only gets 4 items as this is used to convert the quaternion orientation [qw, qx, qy, qz] into C floats
    */
    size_t len;
    mp_obj_t *items;
    int i;

    mp_obj_get_array(array, &len, &items);

    if (len != 4){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Expected 4 values in list/tuple"));
    }

    for (i=0; i < 4; i++){
        output[i] = mp_obj_get_float(items[i]);
    }
}

static void normalize_vector(float* vector, float* output){
    /**
     * Normalizes a 3D vector
    */
    float sum_sq, sum;

    sum_sq = vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2];
    sum = sqrt(sum_sq);

    // Guarding against 0-division error
    if (sum < 1e-10f){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Magnetometer reading too small (vector length < 1e-10 Gauss). Can't normalize vector."));
    }
    else{
        output[0] = vector[0] / sum;
        output[1] = vector[1] / sum;
        output[2] = vector[2] / sum;
    }
}

static void quat_rotate_mag_readings(qmc5883p_obj_t* self, float* quaternion, float* output){
    /**
     * Rotates the magnetometer vector into the world reference frame
     * Only returns x/y values as the z value should (theoretically) be 0 - and it isn't needed anyway
    */
    float qw, qx, qy, qz;
    float normalized_mag[3];

    qw = quaternion[0];
    qx = quaternion[1];
    qy = quaternion[2];
    qz = quaternion[3];

    normalize_vector(self->data, normalized_mag);

    // Using quaternion rotation to find the magnetic north direction vector in the world reference frame
    output[0] = (qw*qw + qx*qx - qy*qy - qz*qz)*normalized_mag[0] + 2*(qx*qy - qw*qz)*normalized_mag[1] + 2*(qx*qz + qw*qy)*normalized_mag[2];
    output[1] = 2*(qx*qy + qw*qz)*normalized_mag[0] + (qw*qw - qx*qx + qy*qy - qz*qz)*normalized_mag[1] + 2*(qy*qz - qw*qx)*normalized_mag[2];
}

static void heading_vector(qmc5883p_obj_t* self, float* quaternion, float* output){
    /**
     * Finds the x/y heading vector for the module
     * I.e. In the world reference frame (or the reference frame of your IMU, if that's where your quaternion is coming from), where is the module physically pointing?
    */
    float qw, qx, qy, qz;

    qw = quaternion[0];
    qx = quaternion[1];
    qy = quaternion[2];
    qz = quaternion[3];

    // Using quaternion rotation to find the magnetometer's heading vector in the world reference frame
    output[0] = (qw*qw + qx*qx - qy*qy - qz*qz)*-1.0f;
    output[1] = (qx*qy + qw*qz)*-2.0f;
}

static uint8_t check_drdy(qmc5883p_obj_t *self){
    /**
     * Checks the QMC5883P's status register to see if the data ready (drdy) bit is set
     * If the drdy bit = 1, there's new sensor data available
    */
    mp_obj_t readfrom_method[2];
    mp_obj_t statusreg_data;
    const uint8_t *statusreg_intdata;
    int counter = 0;
    nlr_buf_t cpu_state;

    mp_load_method(self->i2c_bus, MP_QSTR_readfrom_mem, readfrom_method);
    mp_obj_t arguments[5] = {readfrom_method[0], readfrom_method[1], mp_obj_new_int(self->address), mp_obj_new_int(STATUS_REG), mp_obj_new_int(1)};

    // Checking the DRDY bit of status register
    while (counter < 2){
        // Making sure that the loop times out after two attempts
        counter ++;

        // Reading from the sensor, then checking the bit

        if (nlr_push(&cpu_state) == 0){
            statusreg_data = mp_call_method_n_kw(3, 0, arguments);

            nlr_pop();
        }
        else {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENODEV - Could not communicate with module."));
        }

        statusreg_intdata = mparray_to_int(statusreg_data);

        if (statusreg_intdata[0] & 0x01){
            return 1;
        }
        else {
            mp_hal_delay_ms(5);
        }
    }

    return 0;
}

static void update_data(qmc5883p_obj_t *self){
    /**
     * Updates the raw magnetometer data from the QMC5883P chip
    */
    mp_obj_t mag_data;
    mp_obj_t readfrom_method[2];
    const uint8_t *mag_intdata;
    float xdata, ydata, zdata;
    int flag;
    nlr_buf_t cpu_state;

    mp_load_method(self->i2c_bus, MP_QSTR_readfrom_mem, readfrom_method);

    // Checking if there's new data available
    flag = check_drdy(self);
    if (flag == 0){
        return;
    }

    // Burst reading the 6 data bytes from the sensor
    mp_obj_t data_arguments[5] = {readfrom_method[0], readfrom_method[1], mp_obj_new_int(self->address), mp_obj_new_int(XDATA_REG), mp_obj_new_int(6)};

    if (nlr_push(&cpu_state) == 0){
        mag_data = mp_call_method_n_kw(3, 0, data_arguments);
        nlr_pop();
    }
    else {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENODEV - Could not communicate with module."));
    }

    mag_intdata = mparray_to_int(mag_data);

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

    return;
}

static float list_values_range(float *list, uint16_t length){
    /**
     * Calculates the range between the highest and lowest values in an array
    */
    uint16_t i;
    float max = -INFINITY;
    float min = INFINITY;

    // Iterating through the array to look for the maximum and minimum values
    for (i = 0; i < length; i++){
        if (list[i] > max){
            max = list[i];
        }

        if (list[i] < min){
            min = list[i];
        }
    }

    return max - min;
}

static void calibrationrotation_data(qmc5883p_obj_t *self, float fieldstrength, calibration_data*){
    /**
     * Collects a complete data set for all angles around each magnetometer axis
     * This data can then be used to calibrate the magnetometer
    */
    uint8_t xcomplete = 0, ycomplete = 0, zcomplete = 0;
    uint16_t xcounter = 0, ycounter = 0, zcounter = 0;
    float *xdata = NULL;
    float *ydata = NULL;
    float *zdata = NULL;

    log_func("Begin compass rotation\n");

    // This loops for as long as all the axes don't have complete data
    while (!(xcomplete && ycomplete && zcomplete)){
        update_data(self);

        // If each axis has incomplete data, it reallocates memory to the data arrays and adds the data points to the new arrays
        if (xcomplete == 0){
            xcounter ++;
            xdata = (float *)realloc(xdata, xcounter*FLOAT_SIZE);

            if (xdata == NULL){
                // Error: out of memory
                free(ydata);
                free(zdata);
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
            }

            xdata[xcounter-1] = self->data[0];

            // Termination conditions for each axis: determines whether each axis has had a complete rotation and returned to starting point
            if ((list_values_range(xdata, xcounter) > 1.5*fieldstrength) && (fabs(xdata[xcounter-1] - xdata[0]) < 0.1)){
                xcomplete = 1;
                log_func("X-axis complete\n");
            }
        }

        if (ycomplete == 0){
            ycounter ++;
            ydata = (float *)realloc(ydata, ycounter*FLOAT_SIZE);

            if (ydata == NULL){
                // Error: out of memory
                free(xdata);
                free(zdata);
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
            }

            ydata[ycounter-1] = self->data[1];

            if ((list_values_range(ydata, ycounter) > 1.5*fieldstrength) && (fabs(ydata[ycounter-1] - ydata[0]) < 0.1)){
                ycomplete = 1;
                log_func("Y-axis complete\n");
            }
        }

        if (zcomplete == 0){
            zcounter ++;
            zdata = (float *)realloc(zdata, zcounter*FLOAT_SIZE);

            if (zdata == NULL){
                // Error: out of memory
                free(xdata);
                free(ydata);
                mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
            }

            zdata[zcounter-1] = self->data[2];

            if ((list_values_range(zdata, zcounter) > 1.5*fieldstrength) && (fabs(zdata[zcounter-1] - zdata[0]) < 0.1)){
                zcomplete = 1;
                log_func("Z-axis complete\n");
            }
        }

        mp_hal_delay_ms(10);
    }

    // Putting all the collected data into the struct, which is then returned
    output->xdata = xdata;
    output->ydata = ydata;
    output->zdata = zdata;

    output->xlength = xcounter;
    output->ylength = ycounter;
    output->zlength = zcounter;
}

static uint8_t is_in_array(float* array, uint16_t length, float item){
    /**
     * Utility to confirm whether or not a given item is present in an array
    */
    uint8_t i;

    // Utility to check if an item appears in an array
    for (i = 0; i < length; i++){
        if (array[i] == item){
            return 1;
        }
    }

    return 0;
}

static void max_min_average_array(float* array, uint16_t length, uint8_t num_to_average, float* output){
    /**
     * Finds the num_to_average highest and lowest values in an array
     * The code then averages these highest/lowest values, and returns the average highest/lowest
    */
    if (num_to_average > length){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Incorrect parameters - can't find more values than are present in the array"));
    }

    float *highest = malloc(num_to_average * FLOAT_SIZE);
    float *lowest = malloc(num_to_average * FLOAT_SIZE);
    uint16_t i;
    uint8_t j;

    if ((highest == NULL) || (lowest == NULL)){
        // Error: out of memory
        free(highest);
        free(lowest);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - out of memory."));
    }

    output[0] = 0.0f;
    output[1] = 0.0f;

    // Looking for the 5 highest and lowest values in the given array
    for (j = 0; j < num_to_average; j++){
        float l = INFINITY;
        float h = -INFINITY;

        // Finds the highest/lowest values in an array (whilst making sure the values aren't already in the list of the 5 highest/lowest values)
        for (i = 0; i < length; i++){
            if ((array[i] > h) && (is_in_array(highest, j, array[i]) == 0)){
                h = array[i];
            }

            if ((array[i] < l) && (is_in_array(lowest, j, array[i]) == 0)){
                l = array[i];
            }
        }

        highest[j] = h;
        lowest[j] = l;
    }

    // Sums the arrays and figures out the average highest/lowest value
    for (i = 0; i < num_to_average; i++){
        output[0] += highest[i];
        output[1] += lowest[i];
    }

    output[0] /= num_to_average;
    output[1] /= num_to_average;

    free(highest);
    free(lowest);
}

mp_obj_t getdata_raw(mp_obj_t self_in){
    /**
     * Function to be called from python
     * Returns a python array of the raw [x, y, z] magnetometer data (in Gauss)
    */
    mp_obj_t data_array[3];

    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);

    update_data(self);

    // Creating an array of python floats to return
    data_array[0] = mp_obj_new_float(self->data[0]);
    data_array[1] = mp_obj_new_float(self->data[1]);
    data_array[2] = mp_obj_new_float(self->data[2]);

    return mp_obj_new_list(3, data_array);
}
static MP_DEFINE_CONST_FUN_OBJ_1(qmc5883p_getdata_raw_obj, getdata_raw);

mp_obj_t compass_2d(mp_obj_t self_in, mp_obj_t dec){
    /**
     * Basic compass function
     * Calculates heading based on a basic atan2() of the x/y magnetometer data - no tilt compensation
     * Requires you to pass in a declination value
    */
    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);
    float declination = mp_obj_get_float(dec);
    float heading;
    uint16_t heading_rounded;

    update_data(self);

    // Calculating heading
    heading = (atan2f(self->data[1], -self->data[0]) * RAD_TO_DEG) - declination;

    // Ensuring heading goes from 0 -> 360 degrees
    if (heading > 360.0f){
        heading -= 360.0f;
    }
    else if (heading < 0.0f){
        heading += 360.0f;
    }

    // Rounding heading to nearest degree
    heading_rounded = (uint16_t)(heading+0.5f);

    return mp_obj_new_int_from_uint(heading_rounded);
}
static MP_DEFINE_CONST_FUN_OBJ_2(qmc5883p_compass_2d_obj, compass_2d);

mp_obj_t compass_3d(mp_obj_t self_in, mp_obj_t quaternion, mp_obj_t dec){
    /**
     * Advanced compass function
     * Contains full pitch/roll compensation algorithm for the compass - so it doesn't matter how the module is oriented, you get the correct heading value
     * Requires orientation input as a quaternion [qw, qx, qy, qz], and declination value input
    */
    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);
    float declination = mp_obj_get_float(dec);
    float dotproduct, crossproduct_z, heading, headingvector[2], mag_directionvector[2], quat[4];
    uint16_t heading_rounded;
    nlr_buf_t cpu_state;

    // Error handling 
    // If any of these functions fail, the required memory will be tidied up and then the error code returned
    if (nlr_push(&cpu_state) == 0){
        mparray_to_float(quaternion, quat);

        update_data(self);

        quat_rotate_mag_readings(self, quat, mag_directionvector);
        heading_vector(self, quat, headingvector);

        nlr_pop();
    }
    else {
        nlr_jump(cpu_state.ret_val);
    }

    // Heading calc maths: cross product = |a|*|b|*sin(theta), dot product = |a|*|b|*cos(theta)
    // So atan(crossproduct/dotproduct)=atan(sin(theta)/cos(theta))=atan(tan(theta))=theta
    // I.e. this code finds the angle between the magnetic north direction vector, and the module's heading_vector
    dotproduct = mag_directionvector[0]*headingvector[0] + mag_directionvector[1]*headingvector[1];
    crossproduct_z = mag_directionvector[0]*headingvector[1] - mag_directionvector[1]*headingvector[0];

    heading = (atan2f(crossproduct_z, dotproduct) * RAD_TO_DEG) - declination;

    // Ensuring heading goes from 0->360 degrees
    if (heading > 360.0f){
        heading -= 360.0f;
    }
    else if (heading < 0.0f){
        heading += 360.0f;
    }

    heading_rounded = (uint16_t)(heading+0.5f);

    return mp_obj_new_int_from_uint(heading_rounded);
}
static MP_DEFINE_CONST_FUN_OBJ_3(qmc5883p_compass_3d_obj, compass_3d);

mp_obj_t calibrate(mp_obj_t self_in){
    /**
     * Magnetometer calibration function - calibrates for hard and soft iron effects
     * The function prints an output (to REPL) to let you know where it is in calibration
     * When you've called the calibration function, you then have to rotate the module 360 degrees around any two of the x, y, or z axes until the calibration completes
    */
    qmc5883p_obj_t *self = MP_OBJ_TO_PTR(self_in);

    calibration_data data;
    float fieldstrength_gauss = 0.0f;
    float xoffset, yoffset, zoffset, avg_offset;
    float x_maxmin[2], y_maxmin[2], z_maxmin[2];
    uint8_t i;

    log_func("Calibrating...\n");

    for (i = 0; i < 20; i++){
        update_data(self);
        
        // Summing up the field strengths
        fieldstrength_gauss += sqrt(self->data[0]*self->data[0] + self->data[1]*self->data[1] + self->data[2]*self->data[2]);

        mp_hal_delay_ms(5);
    }

    // Calculating the average field strength
    fieldstrength_gauss /= 20;

    log_func("Local magnetic field strength determined\n");

    // Getting a complete axis of data from each magnetometer axis
    calibrationrotation_data(self, fieldstrength_gauss, &data);

    // Working out the average of the 5 highest/lowest values in each axis' data array
    max_min_average_array(data.xdata, data.xlength, 5, x_maxmin);
    max_min_average_array(data.ydata, data.ylength, 5, y_maxmin);
    max_min_average_array(data.zdata, data.zlength, 5, z_maxmin);

    // Calculating the hard offset calibration values
    self->hardcal[0] = (x_maxmin[0] + x_maxmin[1])/2;
    self->hardcal[1] = (y_maxmin[0] + y_maxmin[1])/2;
    self->hardcal[2] = (z_maxmin[0] + z_maxmin[1])/2;

    log_func("Hard iron calibration calculated\n");

    // Calculating the soft offset calibration values
    xoffset = (x_maxmin[0] - x_maxmin[1])/2;
    yoffset = (y_maxmin[0] - y_maxmin[1])/2;
    zoffset = (z_maxmin[0] - z_maxmin[1])/2;
    avg_offset = (xoffset + yoffset + zoffset)/3;

    self->softcal[0] = avg_offset/xoffset;
    self->softcal[1] = avg_offset/yoffset;
    self->softcal[2] = avg_offset/zoffset;

    log_func("Soft iron calibration calculated\n");

    free(data.xdata);
    free(data.ydata);
    free(data.zdata);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(qmc5883p_calibrate_obj, calibrate);

/**
 * Code here exposes the module functions above to micropython as an object
*/

// Defining the functions that are exposed to micropython
static const mp_rom_map_elem_t qmc5883p_locals_dict_table[] = {
    {MP_ROM_QSTR(MP_QSTR_getdata_raw), MP_ROM_PTR(&qmc5883p_getdata_raw_obj)},
    {MP_ROM_QSTR(MP_QSTR_compass_2d), MP_ROM_PTR(&qmc5883p_compass_2d_obj)},
    {MP_ROM_QSTR(MP_QSTR_compass_3d), MP_ROM_PTR(&qmc5883p_compass_3d_obj)},
    {MP_ROM_QSTR(MP_QSTR_calibrate), MP_ROM_PTR(&qmc5883p_calibrate_obj)},
};
static MP_DEFINE_CONST_DICT(qmc5883p_locals_dict, qmc5883p_locals_dict_table);

// Overall module definition
MP_DEFINE_CONST_OBJ_TYPE(
    qmc5883p_type,
    MP_QSTR_qmc5883p,
    MP_TYPE_FLAG_NONE,
    make_new, qmc5883p_make_new,
    locals_dict, &qmc5883p_locals_dict
);

// Defining global constants
static const mp_rom_map_elem_t qmc5883p_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__) , MP_ROM_QSTR(MP_QSTR_qmc5883p) },
    { MP_ROM_QSTR(MP_QSTR_QMC5883P), MP_ROM_PTR(&qmc5883p_type) },
};
static MP_DEFINE_CONST_DICT(qmc5883p_globals_table, qmc5883p_module_globals_table);

// Creating module object
const mp_obj_module_t qmc5883p_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&qmc5883p_globals_table,
};

MP_REGISTER_MODULE(MP_QSTR_qmc5883p, qmc5883p_module);