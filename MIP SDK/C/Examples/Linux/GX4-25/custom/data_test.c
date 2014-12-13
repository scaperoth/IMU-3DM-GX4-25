/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// If file is being used from remote computer, use the command
/// make copy to send the created files to the server. Otherwise simply use make
///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "data_test.h"

int main(int argc, char *argv[])
{
    ///
    //Verify the command line arguments
    ///

    if (argc != NUM_COMMAND_LINE_ARGUMENTS)
    {
        print_command_line_usage();
        return -1;
    }

    print_header();

    //Convert the arguments
    com_port = atoi(argv[1]);
    baudrate = atoi(argv[2]);


    connect_to_imu(argc, com_port, baudrate);

    set_direct_mode();
    ping_device();


    //reset_filter();

    //print_sensor_frame_transformation();
    mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback);
    //return -1;
    //
    ////Enable the output of data statistics
    enable_data_stats_output = 0;

    set_ahrs_format();

    while (1)
    {
        while (mip_3dm_cmd_poll_ahrs(&device_interface, MIP_3DM_POLLING_ENABLE_ACK_NACK, data_stream_format_num_entries, data_stream_format_descriptors) != MIP_INTERFACE_OK) {}
        printf("\ncallback accel 1, 2, 3: %f | %f | %f\n", curr_ahrs_accel.scaled_accel[0], curr_ahrs_accel.scaled_accel[1], curr_ahrs_accel.scaled_accel[2]);
        printf("\ncallback gyro 1, 2, 3: %f | %f | %f\n", curr_ahrs_gyro.scaled_gyro[0], curr_ahrs_gyro.scaled_gyro[1], curr_ahrs_gyro.scaled_gyro[2]);

        Sleep(100);
    }

    print_exit_message();

    return 0;
}



//printf("Returned angles: %f roll %f pitch %f yaw\n", readback_angles[0], readback_angles[1], readback_angles[2]);

int connect_to_imu(int main_arg1, u32 com_port, u32 baudrate)
{
    ///
    //Initialize the interface to the device
    ///
    if (mip_interface_init(com_port, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
        return -1;

}

void set_direct_mode()
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // IMU-Direct Mode Testing
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    com_mode = MIP_SDK_GX4_25_IMU_DIRECT_MODE;

    ///
    //Set communication mode
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Attempting to set communications mode to IMU Direct mode\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK) {}

    ///
    //Verify device mode setting
    ///

    while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK) {}

    if (com_mode == MIP_SDK_GX4_25_IMU_DIRECT_MODE)
    {

        printf("Communications mode IMU Direct.\n");

        printf("\n\n");
        Sleep(1500);


    }
}

/**
 * [idle_device description]
 */
void idle_device()
{
    ///
    //Put the GX4-25 into idle mode
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Idling Device\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK) {}

    printf("\n\n");
    Sleep(1500);
}


/**
 * [ping_device description]
 */
void ping_device()
{
    ///
    //Try to ping the GX4-25
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Pinging Device\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK) {}

    printf("\n\n");
    Sleep(1500);
}

/**
 * [print_device_info description]
 */
void print_device_info()
{
    ///
    //Get the device information
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting Device Information\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_get_device_info(&device_interface, &device_info) != MIP_INTERFACE_OK) {}

    printf("\n\nDevice Info:\n");
    printf("---------------------------------------------\n");

    memcpy(temp_string, device_info.model_name, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
    printf("Model Name       => %s\n", temp_string);

    memcpy(temp_string, device_info.model_number, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
    printf("Model Number     => %s\n", temp_string);

    memcpy(temp_string, device_info.serial_number, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
    printf("Serial Number    => %s\n", temp_string);

    memcpy(temp_string, device_info.lotnumber, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
    printf("Lot Number       => %s\n", temp_string);

    memcpy(temp_string, device_info.device_options, BASE_DEVICE_INFO_PARAM_LENGTH * 2);
    printf("Options          => %s\n", temp_string);

    printf("Firmware Version => %d.%d.%.2d\n\n", (device_info.firmware_version) / 1000,
           (device_info.firmware_version) % 1000 / 100,
           (device_info.firmware_version) % 100);

    printf("\n\n");
    Sleep(1500);
}

/**
 * [print_supported_descriptors description]
 */
void print_supported_descriptors()
{
    ///
    //Get the supported descriptors
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting Supported descriptors\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_get_device_supported_descriptors(&device_interface, (u8 *)device_descriptors, &device_descriptors_size) != MIP_INTERFACE_OK) {}

    printf("\n\nSupported descriptors:\n\n");

    for (i = 0; i < device_descriptors_size / 2; i++)
    {
        printf("Descriptor Set: %02x, Descriptor: %02x\n", device_descriptors[i] >> 8, device_descriptors[i] & 0xFF);
        Sleep(100);
    }

    printf("\n\n");
    Sleep(1500);
}

/**
 * [print_built_in_test description]
 */
void print_built_in_test()
{
    ///
    //Peform a built-in-test
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Running Built In Test\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_base_cmd_built_in_test(&device_interface, &bit_result) != MIP_INTERFACE_OK) {}

    printf("\nBIT Result (should be 0x00000000) => 0x%08x\n\n", bit_result);

    printf("\n\n");
    Sleep(1500);
}

void set_ahrs_format()
{
    ///
    //Setup the AHRS datastream format
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Setting the AHRS message format\n");
    printf("----------------------------------------------------------------------\n\n");

    data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
    data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;

    data_stream_format_decimation[0]  = 0x64;
    data_stream_format_decimation[1]  = 0x64;

    data_stream_format_num_entries = 2;

    while (mip_3dm_cmd_ahrs_message_format(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries, data_stream_format_descriptors, data_stream_format_decimation) != MIP_INTERFACE_OK) {}

    printf("\n\n");

}

void print_AHRS_datastream_base_rate()
{
    ///
    //Get AHRS Base Rate
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Getting the AHRS datastream base rate\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_3dm_cmd_get_ahrs_base_rate(&device_interface, &base_rate) != MIP_INTERFACE_OK) {}

    printf("\nAHRS Base Rate => %d Hz\n\n", base_rate);

    printf("\n\n");
    Sleep(1500);

}

/**
 * [print_device_status description]
 */
void print_device_status()
{
    ///
    //Get Device Status
    ///


    printf("----------------------------------------------------------------------\n");
    printf("Requesting BASIC Status Report:\n");
    printf("----------------------------------------------------------------------\n\n");

    //Request basic status report
    while (mip_3dm_cmd_hw_specific_imu_device_status(&device_interface, GX4_IMU_MODEL_NUMBER, GX4_IMU_BASIC_STATUS_SEL, &imu_basic_field) != MIP_INTERFACE_OK) {}

    printf("Model Number: \t\t\t\t\t%04u\n", imu_basic_field.device_model);
    printf("Status Selector: \t\t\t\t%s\n", imu_basic_field.status_selector == GX4_IMU_BASIC_STATUS_SEL ? "Basic Status Report" : "Diagnostic Status Report");
    printf("Status Flags: \t\t\t\t\t0x%08x\n", imu_basic_field.status_flags);
    printf("System Millisecond Timer Count: \t\t%llu ms\n\n", imu_basic_field.system_timer_ms);

    printf("Requesting DIAGNOSTIC Status Report:\n");

    //Request diagnostic status report
    while (mip_3dm_cmd_hw_specific_imu_device_status(&device_interface, GX4_IMU_MODEL_NUMBER, GX4_IMU_DIAGNOSTICS_STATUS_SEL, &imu_diagnostic_field) != MIP_INTERFACE_OK) {}

    printf("Model Number: \t\t\t\t\t%04u\n", imu_diagnostic_field.device_model);
    printf("Status Selector: \t\t\t\t%s\n", imu_diagnostic_field.status_selector == GX4_IMU_BASIC_STATUS_SEL ? "Basic Status Report" : "Diagnostic Status Report");
    printf("Status Flags: \t\t\t\t\t0x%08x\n", imu_diagnostic_field.status_flags);
    printf("System Millisecond Timer Count: \t\t%llu ms\n", imu_diagnostic_field.system_timer_ms);
    printf("Magnetometer: \t\t\t\t\t%s\n", imu_diagnostic_field.has_mag == 1 ? "DETECTED" : "NOT-DETECTED");
    printf("Pressure Sensor: \t\t\t\t%s\n", imu_diagnostic_field.has_pressure == 1 ? "DETECTED" : "NOT-DETECTED");
    printf("Gyro Range Reported: \t\t\t\t%u deg/s\n", imu_diagnostic_field.gyro_range);
    printf("Accel Range Reported: \t\t\t\t%u G\n", imu_diagnostic_field.accel_range);
    printf("Magnetometer Range Reported: \t\t\t%f Gs\n", imu_diagnostic_field.mag_range);
    printf("Pressure Range Reported: \t\t\t%f hPa\n", imu_diagnostic_field.pressure_range);
    printf("Measured Internal Temperature: \t\t\t%f degrees C\n", imu_diagnostic_field.temp_degc);
    printf("Last Temperature Measured: \t\t\t%u ms\n", imu_diagnostic_field.last_temp_read_ms);
    printf("Bad Temperature Sensor Detected: \t\t%s\n", imu_diagnostic_field.temp_sensor_error == 1 ? "TRUE" : "FALSE");
    printf("Number Received GPS Pulse-Per-Second Pulses: \t%u Pulses\n", imu_diagnostic_field.num_gps_pps_triggers);
    printf("Time of Last GPS Pulse-Per-Second Pulse: \t%u ms\n", imu_diagnostic_field.last_gps_pps_trigger_ms);
    printf("Data Streaming Enabled: \t\t\t%s\n", imu_diagnostic_field.stream_enabled == 1 ? "TRUE" : "FALSE");
    printf("Number of Dropped Communication Packets: \t%u packets\n", imu_diagnostic_field.dropped_packets);
    printf("Communications Port Bytes Written: \t\t%u Bytes\n", imu_diagnostic_field.com_port_bytes_written);
    printf("Communications Port Bytes Read: \t\t%u Bytes\n", imu_diagnostic_field.com_port_bytes_read);
    printf("Communications Port Write Overruns: \t\t%u Bytes\n", imu_diagnostic_field.com_port_write_overruns);
    printf("Communications Port Read Overruns: \t\t%u Bytes\n", imu_diagnostic_field.com_port_read_overruns);

    printf("\n\n");
    Sleep(1500);
}

void print_gyro_bias()
{
    ///
    //Capture Gyro Bias
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Performing Gyro Bias capture.\nPlease keep device stationary during the 5 second gyro bias capture interval\n");
    printf("----------------------------------------------------------------------\n\n");

    duration = 5000; //milliseconds

    while (mip_3dm_cmd_capture_gyro_bias(&device_interface, duration, bias_vector) != MIP_INTERFACE_OK) {}

    printf("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n", bias_vector[0], bias_vector[1], bias_vector[2]);

    printf("\n\n");
    Sleep(1500);

}

/**
 * [reset_filter description]
 */
void reset_filter()
{
    ///
    //Reset the filter
    ///

    printf("----------------------------------------------------------------------\n");
    printf("Resetting the Filter\n");
    printf("----------------------------------------------------------------------\n\n");

    while (mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK) {}

    printf("\n\n");
    Sleep(1500);
}

/**
 * [get_roll description]
 */
void print_sensor_frame_transformation()
{
    ///
    //Set/Read the Sensor to Vehicle Frame transformation
    ///

    //printf("----------------------------------------------------------------------\n");
    //printf("Setting the sensor to vehicle frame transformation\n");
    //printf("----------------------------------------------------------------------\n\n");

    angles[0] = 0;
    angles[1] = 0;
    angles[2] = 0;

    while (mip_filter_sensor2vehicle_tranformation(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, angles) != MIP_INTERFACE_OK) {}

    //Read back the transformation
    while (mip_filter_sensor2vehicle_tranformation(&device_interface, MIP_FUNCTION_SELECTOR_READ, readback_angles) != MIP_INTERFACE_OK) {}

    if ((abs(readback_angles[0] - angles[0]) < 0.001) &&
            (abs(readback_angles[1] - angles[1]) < 0.001) &&
            (abs(readback_angles[2] - angles[2]) < 0.001))
    {
        //printf("Transformation successfully set.\n");
    }
    else
    {
        printf("ERROR: Failed to set transformation!!!\n");
        printf("Sent angles:     %f roll %f pitch %f yaw\n", angles[0], angles[1], angles[2]);
        printf("Returned angles: %f roll %f pitch %f yaw\n", readback_angles[0], readback_angles[1], readback_angles[2]);
    }

    printf("Returned angles: %f roll %f pitch %f yaw\n", readback_angles[0], readback_angles[1], readback_angles[2]);



    //printf("\n\nLoading the default sensor to vehicle transformation.\n\n");

    while (mip_filter_sensor2vehicle_tranformation(&device_interface, MIP_FUNCTION_SELECTOR_LOAD_DEFAULT, NULL) != MIP_INTERFACE_OK) {}

    printf("\n\n");
    //Sleep(1500);

}



////////////////////////////////////////////////////////////////////////////////
//
// Print Header
//
////////////////////////////////////////////////////////////////////////////////

void print_header()
{
    printf("\n");
    printf("GX4-25 Test Program\n");
    printf("\n\n");
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Exit Message
//
////////////////////////////////////////////////////////////////////////////////

void print_exit_message()
{
    printf("\n");
    printf("Exiting Test Program\n");
    printf("Goodbye!\n\n");
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Command-Line Help
//
////////////////////////////////////////////////////////////////////////////////

void print_command_line_usage()
{
    printf("\n\n");
    printf("Usage:\n");
    printf("-----------------------------------------------------------------------\n\n");

    printf("   GX4-25_Test [com_port_num] [baudrate]\n");
    printf("\n\n");
    printf("   Example: \"GX4-25_Test 1 115200\", Opens a connection to the \n");
    printf("             GX4-25 on COM1, with a baudrate of 115200.\n");
    printf("\n\n");
    printf("   [ ] - required command input.\n");
    printf("\n-----------------------------------------------------------------------\n");
    printf("\n\n");
}


////////////////////////////////////////////////////////////////////////////////
//
// Print Packet Statistics
//
////////////////////////////////////////////////////////////////////////////////

void print_packet_stats()
{
    if (enable_data_stats_output)
    {
        printf("\r%u FILTER (%u errors)    %u AHRS (%u errors)   Packets", filter_valid_packet_count,  filter_timeout_packet_count + filter_checksum_error_packet_count,
               ahrs_valid_packet_count, ahrs_timeout_packet_count + ahrs_checksum_error_packet_count);
    }
}

////////////////////////////////////////////////////////////////////////////////
//
// FILTER Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch (callback_type)
    {
    ///
    //Handle valid packets
    ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    {
        filter_valid_packet_count++;

        ///
        //Loop through all of the data fields
        ///

        while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {

            ///
            // Decode the field
            ///

            switch (field_header->descriptor)
            {
            ///
            // Estimated Attitude, Euler Angles
            ///

            case MIP_FILTER_DATA_ATT_EULER_ANGLES:
            {
                memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

                //For little-endian targets, byteswap the data field
                mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

            } break;

            default: break;
            }
        }
    } break;


    ///
    //Handle checksum error packets
    ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    {
        filter_checksum_error_packet_count++;
    } break;

    ///
    //Handle timeout packets
    ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT:
    {
        filter_timeout_packet_count++;
    } break;
    default: break;
    }

    print_packet_stats();
}


////////////////////////////////////////////////////////////////////////////////
//
// AHRS Packet Callback
//
////////////////////////////////////////////////////////////////////////////////

void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
    mip_field_header *field_header;
    u8               *field_data;
    u16              field_offset = 0;

    //The packet callback can have several types, process them all
    switch (callback_type)
    {
    ///
    //Handle valid packets
    ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET:
    {
        ahrs_valid_packet_count++;

        ///
        //Loop through all of the data fields
        ///

        while (mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
        {

            ///
            // Decode the field
            ///

            switch (field_header->descriptor)
            {
            ///
            // Scaled Accelerometer
            ///

            case MIP_AHRS_DATA_ACCEL_SCALED:
            {
                memcpy(&curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

                //For little-endian targets, byteswap the data field
                mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

            } break;

            ///
            // Scaled Gyro
            ///

            case MIP_AHRS_DATA_GYRO_SCALED:
            {
                memcpy(&curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

                //For little-endian targets, byteswap the data field
                mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

            } break;

            ///
            // Scaled Magnetometer
            ///

            case MIP_AHRS_DATA_MAG_SCALED:
            {
                memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

                //For little-endian targets, byteswap the data field
                mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

            } break;

            default: break;
            }
        }
    } break;

    ///
    //Handle checksum error packets
    ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
    {
        ahrs_checksum_error_packet_count++;
    } break;

    ///
    //Handle timeout packets
    ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT:
    {
        ahrs_timeout_packet_count++;
    } break;
    default: break;
    }

    print_packet_stats();
}


///////////////////////////////////////////////////////////////////////////////
//
// Device specific Status Acquisition Routines
//
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
//
//! @section DESCRIPTION
//! Requests GX4-IMU Basic or Diagnostic Status Message.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4 IMU (6237)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n
//
//! @section NOTES
//!
//! This function should only be called in IMU Direct Mode.
///////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
{
    gx4_imu_basic_status_field *basic_ptr;
    gx4_imu_diagnostic_device_status_field *diagnostic_ptr;
    u16 response_size = MIP_FIELD_HEADER_SIZE;

    if (status_selector == GX4_IMU_BASIC_STATUS_SEL)
        response_size += sizeof(gx4_imu_basic_status_field);
    else if (status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
        response_size += sizeof(gx4_imu_diagnostic_device_status_field);

    while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK) {}

    if (status_selector == GX4_IMU_BASIC_STATUS_SEL)
    {
        if (response_size != sizeof(gx4_imu_basic_status_field))
            return MIP_INTERFACE_ERROR;
        else if (MIP_SDK_CONFIG_BYTESWAP)
        {
            basic_ptr = (gx4_imu_basic_status_field *)response_buffer;

            byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
            byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
            byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
        }
    }
    else if (status_selector == GX4_IMU_DIAGNOSTICS_STATUS_SEL)
    {

        if (response_size != sizeof(gx4_imu_diagnostic_device_status_field))
            return MIP_INTERFACE_ERROR;
        else if (MIP_SDK_CONFIG_BYTESWAP)
        {
            diagnostic_ptr = (gx4_imu_diagnostic_device_status_field *)response_buffer;

            byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
            byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
            byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
            byteswap_inplace(&diagnostic_ptr->gyro_range, sizeof(diagnostic_ptr->gyro_range));
            byteswap_inplace(&diagnostic_ptr->mag_range, sizeof(diagnostic_ptr->mag_range));
            byteswap_inplace(&diagnostic_ptr->pressure_range, sizeof(diagnostic_ptr->pressure_range));
            byteswap_inplace(&diagnostic_ptr->temp_degc, sizeof(diagnostic_ptr->temp_degc));
            byteswap_inplace(&diagnostic_ptr->last_temp_read_ms, sizeof(diagnostic_ptr->last_temp_read_ms));
            byteswap_inplace(&diagnostic_ptr->num_gps_pps_triggers, sizeof(diagnostic_ptr->num_gps_pps_triggers));
            byteswap_inplace(&diagnostic_ptr->last_gps_pps_trigger_ms, sizeof(diagnostic_ptr->last_gps_pps_trigger_ms));
            byteswap_inplace(&diagnostic_ptr->dropped_packets, sizeof(diagnostic_ptr->dropped_packets));
            byteswap_inplace(&diagnostic_ptr->com_port_bytes_written, sizeof(diagnostic_ptr->com_port_bytes_written));
            byteswap_inplace(&diagnostic_ptr->com_port_bytes_read, sizeof(diagnostic_ptr->com_port_bytes_read));
            byteswap_inplace(&diagnostic_ptr->com_port_write_overruns, sizeof(diagnostic_ptr->com_port_write_overruns));
            byteswap_inplace(&diagnostic_ptr->com_port_read_overruns, sizeof(diagnostic_ptr->com_port_read_overruns));
        }
    }
    else
        return MIP_INTERFACE_ERROR;

    return MIP_INTERFACE_OK;
}


///////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
//
//! @section DESCRIPTION
//! Requests GX4-25 Basic or Diagnostic Status Message.
//
//! @section DETAILS
//!
//! @param [in] mip_interface *device_interface - pointer to the mip interface structure.
//! @param [in] u16 model_number - LORD Microstrain Sensing Systems model number for GX4-25 (6236)
//! @param [in] u8 status selector - specifies which type of status message is being requested.
//! @paran [out] u8 *response_buffer - pointer to the location to store response bytes.
//
//! @retval MIP_INTERFACE_ERROR  Interface not initialized or device not in IMU Direct Mode.\n
//! @retval MIP_INTERFACE_OK     Status message successfully recieved.\n
//
//! @section NOTES
//!
///////////////////////////////////////////////////////////////////////////////

u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer)
{

    gx4_25_basic_status_field *basic_ptr;
    gx4_25_diagnostic_device_status_field *diagnostic_ptr;
    u16 response_size = MIP_FIELD_HEADER_SIZE;

    if (status_selector == GX4_25_BASIC_STATUS_SEL)
        response_size += sizeof(gx4_25_basic_status_field);
    else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
        response_size += sizeof(gx4_25_diagnostic_device_status_field);

    while (mip_3dm_cmd_device_status(device_interface, model_number, status_selector, response_buffer, &response_size) != MIP_INTERFACE_OK) {}

    if (status_selector == GX4_25_BASIC_STATUS_SEL)
    {

        if (response_size != sizeof(gx4_25_basic_status_field))
            return MIP_INTERFACE_ERROR;
        else if (MIP_SDK_CONFIG_BYTESWAP)
        {
            basic_ptr = (gx4_25_basic_status_field *)response_buffer;

            byteswap_inplace(&basic_ptr->device_model, sizeof(basic_ptr->device_model));
            byteswap_inplace(&basic_ptr->status_flags, sizeof(basic_ptr->status_flags));
            byteswap_inplace(&basic_ptr->system_timer_ms, sizeof(basic_ptr->system_timer_ms));
        }

    }
    else if (status_selector == GX4_25_DIAGNOSTICS_STATUS_SEL)
    {
        if (response_size != sizeof(gx4_25_diagnostic_device_status_field))
            return MIP_INTERFACE_ERROR;
        else if (MIP_SDK_CONFIG_BYTESWAP)
        {
            diagnostic_ptr = (gx4_25_diagnostic_device_status_field *)response_buffer;

            byteswap_inplace(&diagnostic_ptr->device_model, sizeof(diagnostic_ptr->device_model));
            byteswap_inplace(&diagnostic_ptr->status_flags, sizeof(diagnostic_ptr->status_flags));
            byteswap_inplace(&diagnostic_ptr->system_timer_ms, sizeof(diagnostic_ptr->system_timer_ms));
            byteswap_inplace(&diagnostic_ptr->imu_dropped_packets, sizeof(diagnostic_ptr->imu_dropped_packets));
            byteswap_inplace(&diagnostic_ptr->filter_dropped_packets, sizeof(diagnostic_ptr->filter_dropped_packets));
            byteswap_inplace(&diagnostic_ptr->com1_port_bytes_written, sizeof(diagnostic_ptr->com1_port_bytes_written));
            byteswap_inplace(&diagnostic_ptr->com1_port_bytes_read, sizeof(diagnostic_ptr->com1_port_bytes_read));
            byteswap_inplace(&diagnostic_ptr->com1_port_write_overruns, sizeof(diagnostic_ptr->com1_port_write_overruns));
            byteswap_inplace(&diagnostic_ptr->com1_port_read_overruns, sizeof(diagnostic_ptr->com1_port_read_overruns));
            byteswap_inplace(&diagnostic_ptr->imu_parser_errors, sizeof(diagnostic_ptr->imu_parser_errors));
            byteswap_inplace(&diagnostic_ptr->imu_message_count, sizeof(diagnostic_ptr->imu_message_count));
            byteswap_inplace(&diagnostic_ptr->imu_last_message_ms, sizeof(diagnostic_ptr->imu_last_message_ms));
        }
    }
    else
        return MIP_INTERFACE_ERROR;

    return MIP_INTERFACE_OK;

}
