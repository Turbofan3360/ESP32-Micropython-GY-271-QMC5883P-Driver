#include "qmc5883p.h"

mp_obj_t qmc5883p_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    /**
     * This function initialises a new driver instance. It initialises the I2C bus and adds the magnetometer to it
     * It then calls magnetometer_setup to configure the QMC5883P's config registers as required.
    */
    gpio_num_t scl_pin, sda_pin;
    int8_t port;
    i2c_port_num_t i2c_port;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;
    esp_err_t err = ESP_ERR_INVALID_STATE;

    // Defining the allowed arguments, and setting default values for I2C port/address. Can be modified as keyword arguments
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_scl, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_sda, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_i2c_port, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_I2C_PORT_NUM} },
    };

    // Checking arguments
    mp_arg_val_t parsed_args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, MP_ARRAY_SIZE(allowed_args), allowed_args, parsed_args);

    // Extracting arguments
    scl_pin = parsed_args[0].u_int;
    sda_pin = parsed_args[1].u_int;
    port = parsed_args[2].u_int;

    // Ensuring I2C port number and pin numbers are valid
    if (!GPIO_IS_VALID_OUTPUT_GPIO(scl_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(sda_pin)){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid SCL or SDA pin number"));
    }

    if ((port < -1) || (port > 1)){
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Invalid I2C port number"));
    }

    // If port is not set to autoselect:
    if (port != -1){
        // Trying to pull handle for the bus (if it's already been initialized)
        err = i2c_master_get_bus_handle(port, &bus_handle);
    }

    // If there's no already initialized bus handle or port is set to autoselect, then create a bus:
    if (err == ESP_ERR_INVALID_STATE){
        // Setting I2C port value
        if (port == -1){
            i2c_port = -1;
        }
        else if (port == 0){
            i2c_port = I2C_NUM_0;
        }
        else if (port == 1){
            i2c_port = I2C_NUM_1;
        }

        // Configuring ESP-IDF I2C bus object
        i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_port,
            .scl_io_num = scl_pin,
            .sda_io_num = sda_pin,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };

        // Creating the bus
        err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);

        if (err != ESP_OK){
            mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error initialising I2C bus: %s"), esp_err_to_name(err));
        }
    }

    // Adding the QMC5883P slave device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883P_I2C_ADDRESS,
        .scl_speed_hz = 400000,
    };

    // Installing this to the bus
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &device_handle);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Error adding device to I2C bus: %s"), esp_err_to_name(err));
    }

    // Creating the "self" object for this driver
    qmc5883p_obj_t* self = m_new_obj(qmc5883p_obj_t);

    // Initialising the required data in the "self" object
    self->base.type = &qmc5883p_type;
    self->bus_handle = bus_handle;
    self->device_handle = device_handle;
    self->i2c_address = QMC5883P_I2C_ADDRESS;

    self->data[0] = self->data[1] = self->data[2] = 0.0f;
    self->softcal[0] = self->softcal[1] = self->softcal[2] = 1.0f;
    self->hardcal[0] = self->hardcal[1] = self->hardcal[2] = 0.0f;

    // 1ms delay to ensure the chip powers up properly
    wait_micro_s(1000);

    magnetometer_setup(self);

    return MP_OBJ_FROM_PTR(self);
}

static void magnetometer_setup(qmc5883p_obj_t *self){
    /**
     * This function configures the QMC5883P chip's registers to the required settings (described below)
    */
    uint8_t write_data[2];
    esp_err_t err;

    // Probing to check there's actually a device there
    err = i2c_master_probe(self->bus_handle, self->i2c_address, 100);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("QMC5883P device not found on I2C bus: %s"), esp_err_to_name(err));
    }

    // Configuring module in normal power mode, 200Hz output rate, oversampling=4, downsampling=0
    // Writing to first control register
    write_data[0] = CONTROL_1_REG;
    write_data[1] = 0x1D;
    err = i2c_master_transmit(self->device_handle, write_data, 2, 100);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Unable to write to sensor configuration registers: %s"), esp_err_to_name(err));
    }

    // Writing to second control register
    write_data[0] = CONTROL_2_REG;
    write_data[1] = 0x0C;
    err = i2c_master_transmit(self->device_handle, write_data, 2, 100);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Unable to write to sensor configuration registers: %s"), esp_err_to_name(err));
    }
}

static void wait_micro_s(uint32_t micro_s_delay){
    /**
     * Function to delay by a certain number of microseconds
    */
    uint64_t start = esp_timer_get_time();

    while (esp_timer_get_time() - start < micro_s_delay){}

    return;
}

static void log_func(const char *log_string){
    /**
     * Basic logging function - currently just prints to the REPL, but can be adapted to log to other places (e.g. log to a file) if needed
    */
    mp_printf(&mp_plat_print, "%s", log_string);
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
    sum = sqrtf(sum_sq);

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
    uint8_t attemtps = 0, write_data[1], read_data[1];
    esp_err_t err;

    write_data[0] = STATUS_REG;

    while (attemtps < 2){
        err = i2c_master_transmit_receive(self->device_handle, write_data, 1, read_data, 1, 20);

        if (err != ESP_OK){
            mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Unable to read QMC5883P status register: %s"), esp_err_to_name(err));
        }

        if (read_data[0] & 0x01){
            return 1;
        }

        // 0.5ms delay
        wait_micro_s(500);
        attemtps ++;
    }

    return 0;
}

static void update_data(qmc5883p_obj_t *self){
    /**
     * Updates the raw magnetometer data from the QMC5883P chip
    */
    uint8_t write_data[1], read_data[6];
    float xdata, ydata, zdata;
    esp_err_t err;

    write_data[0] = XDATA_REG;

    // Checking if there's new data available
    if (check_drdy(self) == 0){
        return;
    }

    // Burst reading the 6 bytes from the sensor
    err = i2c_master_transmit_receive(self->device_handle, write_data, 1, read_data, 6, 20);

    if (err != ESP_OK){
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("Unable to read QMC5883P data register: %s"), esp_err_to_name(err));
    }

    // Decoding the data into floats
    xdata = (int16_t)((read_data[1] << 8) | read_data[0]) / 15000.0f;
    ydata = (int16_t)((read_data[3] << 8) | read_data[2]) / 15000.0f;
    zdata = (int16_t)((read_data[5] << 8) | read_data[4]) / 15000.0f;

    xdata = (xdata - self->hardcal[0]) * self->softcal[0];
    ydata = (ydata - self->hardcal[1]) * self->softcal[1];
    zdata = (zdata - self->hardcal[2]) * self->softcal[2];

    self->data[0] = xdata;
    self->data[1] = ydata;
    self->data[2] = zdata;

    return;
}

static void calibrationrotation_data(qmc5883p_obj_t *self, float fieldstrength, calibration_data* output){
    /**
     * Collects a complete data set for all angles around each magnetometer axis
     * This data can then be used to calibrate the magnetometer
    */
    uint8_t axescomplete[3] = {0, 0, 0};
    int16_t headpos[3] = {-1, -1, -1};
    uint16_t listlengths[3] = {0, 0, 0};
    float* xdata = malloc(CALIBRATION_DATA_LIST_LENGTHS*FLOAT_SIZE);
    float* ydata = malloc(CALIBRATION_DATA_LIST_LENGTHS*FLOAT_SIZE);
    float* zdata = malloc(CALIBRATION_DATA_LIST_LENGTHS*FLOAT_SIZE);
    float init_values[3], max_vals[3] = {0, 0, 0}, min_vals[3] = {0, 0, 0};

    // Checking memory allocation
    if (xdata == NULL || ydata == NULL || zdata == NULL){
        free(xdata);
        free(ydata);
        free(zdata);
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ENOMEM - Out of memory"));
    }

    log_func("Begin compass rotation\n");

    // Finding starting magnetometer readings
    update_data(self);
    init_values[0] = self->data[0];
    init_values[1] = self->data[1];
    init_values[2] = self->data[2];

    // This loops for as long as all the axes don't have complete data
    while (!(axescomplete[0] && axescomplete[1] && axescomplete[2])){
        // TODO: FIX MEMORY LEAKING HERE IF update_date RAISES ERROR
        update_data(self);

        // If each axis has incomplete data, it adds a new data point to the data arrays (circular buffer types)
        if (axescomplete[0] == 0){
            headpos[0] = (headpos[0]+1)%CALIBRATION_DATA_LIST_LENGTHS;

            if (listlengths[0] < CALIBRATION_DATA_LIST_LENGTHS){
                listlengths[0] ++;
            }

            xdata[headpos[0]] = self->data[0];

            // Tracking max/min values
            if (self->data[0] > max_vals[0]){
                max_vals[0] = self->data[0];
            }
            else if (self->data[0] < min_vals[0]){
                min_vals[0] = self->data[0];
            }

            // Termination conditions for each axis: determines whether each axis has had a complete rotation and returned to starting point
            if ((max_vals[0]-min_vals[0] > 1.5*fieldstrength) && (fabs(xdata[headpos[0]] - init_values[0]) < 0.1)){
                axescomplete[0] = 1;
                log_func("X-axis complete\n");
            }
        }

        if (axescomplete[1] == 0){
            headpos[1] = (headpos[1]+1)%CALIBRATION_DATA_LIST_LENGTHS;

            if (listlengths[1] < CALIBRATION_DATA_LIST_LENGTHS){
                listlengths[1] ++;
            }

            ydata[headpos[1]] = self->data[1];

            // Tracking max/min values
            if (self->data[1] > max_vals[1]){
                max_vals[1] = self->data[1];
            }
            else if (self->data[1] < min_vals[1]){
                min_vals[1] = self->data[1];
            }

            // Termination conditions for each axis: determines whether each axis has had a complete rotation and returned to starting point
            if ((max_vals[1]-min_vals[1] > 1.5*fieldstrength) && (fabs(ydata[headpos[1]] - init_values[1]) < 0.1)){
                axescomplete[1] = 1;
                log_func("Y-axis complete\n");
            }
        }

        if (axescomplete[2] == 0){
            headpos[2] = (headpos[2]+1)%CALIBRATION_DATA_LIST_LENGTHS;

            if (listlengths[2] < CALIBRATION_DATA_LIST_LENGTHS){
                listlengths[2] ++;
            }

            zdata[headpos[2]] = self->data[2];

            // Tracking max/min values
            if (self->data[2] > max_vals[2]){
                max_vals[2] = self->data[2];
            }
            else if (self->data[2] < min_vals[2]){
                min_vals[2] = self->data[2];
            }

            // Termination conditions for each axis: determines whether each axis has had a complete rotation and returned to starting point
            if ((max_vals[2]-min_vals[2] > 1.5*fieldstrength) && (fabs(zdata[headpos[2]] - init_values[2]) < 0.1)){
                axescomplete[2] = 1;
                log_func("Z-axis complete\n");
            }
        }

        wait_micro_s(10000);
    }

    // Putting all the collected data into the struct, which is then returned
    output->xdata = xdata;
    output->ydata = ydata;
    output->zdata = zdata;

    output->xlength = listlengths[0];
    output->ylength = listlengths[1];
    output->zlength = listlengths[2];
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

static void max_min_average_array(float* array, uint16_t length, float* output){
    /**
     * Finds the 5 highest and lowest values in an array
     * The code then averages these highest/lowest values, and returns the average highest/lowest
    */
    uint8_t num_to_average = 5;

    // Keeping num_to_average safe - even though length < 5 is almost impossible
    if (length < 5){
        num_to_average = length;
    }

    float highest[5], lowest[5];
    uint16_t i;
    uint8_t j;

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

    mparray_to_float(quaternion, quat);

    update_data(self);

    quat_rotate_mag_readings(self, quat, mag_directionvector);
    heading_vector(self, quat, headingvector);

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
        fieldstrength_gauss += sqrtf(self->data[0]*self->data[0] + self->data[1]*self->data[1] + self->data[2]*self->data[2]);

        wait_micro_s(5000);
    }

    // Calculating the average field strength
    fieldstrength_gauss /= 20;

    log_func("Local magnetic field strength determined\n");

    // Getting a complete axis of data from each magnetometer axis
    calibrationrotation_data(self, fieldstrength_gauss, &data);

    // Working out the average of the 5 highest/lowest values in each axis' data array
    max_min_average_array(data.xdata, data.xlength, x_maxmin);
    max_min_average_array(data.ydata, data.ylength, y_maxmin);
    max_min_average_array(data.zdata, data.zlength, z_maxmin);

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
