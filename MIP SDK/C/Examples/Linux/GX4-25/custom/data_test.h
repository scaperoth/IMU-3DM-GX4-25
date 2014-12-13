
#ifndef _GX4_25_TEST_H
#define _GX4_25_TEST_H


///////////////////////////////////////////////////////////////////////////////
//
// Includes
//
///////////////////////////////////////////////////////////////////////////////

#include "mip_sdk.h"
#include "byteswap_utilities.h"
#include "mip_gx4_imu.h"
#include "mip_gx4_25.h"
#include <stdio.h>
#include <unistd.h>

///////////////////////////////////////////////////////////////////////////////
//
// Defines
//
///////////////////////////////////////////////////////////////////////////////

#define MIP_SDK_GX4_25_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX4_25_IMU_DIRECT_MODE    0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

#define Sleep(x) usleep(x*1000.0)

#define DEBUG 1


////////////////////////////////////////////////////////////////////////////////
//
// Globals
//
////////////////////////////////////////////////////////////////////////////////

u8 enable_data_stats_output = 0;

//The primary device interface structure
mip_interface device_interface;

//Packet Counters (valid, timeout, and checksum errors)
u32 filter_valid_packet_count  = 0;
u32 ahrs_valid_packet_count = 0;

u32 filter_timeout_packet_count  = 0;
u32 ahrs_timeout_packet_count = 0;

u32 filter_checksum_error_packet_count  = 0;
u32 ahrs_checksum_error_packet_count = 0;

//Example data field storage

//AHRS
mip_ahrs_scaled_gyro  curr_ahrs_gyro;
mip_ahrs_scaled_accel curr_ahrs_accel;
mip_ahrs_scaled_mag   curr_ahrs_mag;

//FILTER
mip_filter_attitude_euler_angles curr_filter_angles;

//MAIN
u32 com_port, baudrate;
base_device_info_field device_info;
u8  temp_string[20] = {0};
u32 bit_result;
u8  enable = 1;
u8  data_stream_format_descriptors[10];
u16 data_stream_format_decimation[10];
u8  data_stream_format_num_entries = 0;
u8  readback_data_stream_format_descriptors[10] = {0};
u16 readback_data_stream_format_decimation[10]  = {0};
u8  readback_data_stream_format_num_entries     =  0;
u16 base_rate = 0;
u16 device_descriptors[128]  = {0};
u16 device_descriptors_size  = 128 * 2;
s16 i;
u16 j;
u8  com_mode = 0;
u8  readback_com_mode = 0;
float angles[3]             = {0};
float readback_angles[3]    = {0};
float offset[3]             = {0};
float readback_offset[3]    = {0};
float hard_iron[3]          = {0};
float hard_iron_readback[3] = {0};
float soft_iron[9]          = {0};
float soft_iron_readback[9] = {0};
u16 estimation_control   = 0, estimation_control_readback = 0;
u8  heading_source = 0;
u8  auto_init      = 0;
float noise[3]          = {0};
float readback_noise[3] = {0};
float beta[3]                 = {0};
float readback_beta[3]        = {0};
mip_low_pass_filter_settings filter_settings;
float bias_vector[3]      = {0};
u16 duration = 0;
gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
gx4_imu_basic_status_field imu_basic_field;
gx4_25_diagnostic_device_status_field diagnostic_field;
gx4_25_basic_status_field basic_field;
mip_filter_external_heading_update_command external_heading_update;
mip_filter_zero_update_command zero_update_control, zero_update_readback;
mip_filter_external_heading_with_time_command external_heading_with_time;
mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

u8  declination_source_command, declination_source_readback;

mip_filter_accel_magnitude_error_adaptive_measurement_command        accel_magnitude_error_command, accel_magnitude_error_readback;
mip_filter_magnetometer_magnitude_error_adaptive_measurement_command mag_magnitude_error_command, mag_magnitude_error_readback;
mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command mag_dip_angle_error_command, mag_dip_angle_error_readback;
u8 reference_position_enable_command, reference_position_enable_readback;
double reference_position_command[3], reference_position_readback[3];


////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

//Help Functions
void print_header();
void print_command_line_usage();
void print_exit_message();
void print_packet_stats();

//MIP Parser Packet Callback Functions
void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

//Hardware specific status functions
u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);
u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);


//IMU Functions
int connect_to_imu(int main_arg1, u32 com_port, u32 baudrate);
void set_direct_mode();
void idle_device();
void ping_device();
void print_built_in_test();
void print_device_info();
void print_supported_descriptors();
void print_AHRS_datastream_base_rate();
void print_device_status();
void print_gyro_bias();
void reset_filter();
void print_sensor_frame_transformation();



#endif
