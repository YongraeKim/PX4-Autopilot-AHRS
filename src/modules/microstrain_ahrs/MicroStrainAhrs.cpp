/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file MicroStrainAhrs.cpp
 *
 * Matlab CSV / ASCII format interface at 921600 baud, 8 data bits,
 * 1 stop bit, no parity
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include "MicroStrainAhrs.hpp"

#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <termios.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>


#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

MicroStrainAhrs::MicroStrainAhrs() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	parameters_updated();
}

MicroStrainAhrs::~MicroStrainAhrs()
{
	perf_free(_loop_perf);
}

void
MicroStrainAhrs::parameters_updated()
{

}

bool
MicroStrainAhrs::init()
{
	ScheduleOnInterval(5000);

	return true;
}
unsigned char rbuf[129] = {0,};
void
MicroStrainAhrs::Run()
{
	if (should_exit()) {
		_sensor_combined_sub.unregisterCallback();
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	if(serial_fd<0){
		const char* uart_name="/dev/ttyS6";
		serial_fd = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
		tcflush(serial_fd, TCIOFLUSH);
		int speed = 230400;
		PX4_INFO("serial fd : %d",serial_fd);
		if (serial_fd < 0) {
			PX4_INFO("failed to open port: /dev/ttyS6");
			return ;
		}
		struct termios uart_config;
		int termios_state;
		/* Initialize the uart config */
		if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
			PX4_INFO("ERR GET CONF %s", uart_name);
			::close(serial_fd);
			return ;
		}
		/* Clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			PX4_INFO("ERR SET BAUD %s: %d\n", uart_name, termios_state);
			::close(serial_fd);
			return ;
		}
		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_INFO("ERR: %d (cfsetospeed)\n", termios_state);
			return ;
		}
		if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
			PX4_INFO("ERR SET CONF %s\n", uart_name);
			::close(serial_fd);
			return ;
		}
		PX4_INFO("serial port open at /dev/ttyS6(GPS2port)");
	}
	_callback_registered = _sensor_combined_sub.registerCallback();
	if (!_callback_registered) {
		ScheduleDelayed(10_ms);
		return;
	}
	_callback_registered = _vehicle_attitude_sub.registerCallback();
	if (!_callback_registered) {
		ScheduleDelayed(10_ms);
		return;
	}
	sensor_combined_s sensor_combined;
	bool imu_updated=false;
	imu_updated = _sensor_combined_sub.update(&sensor_combined);
	if (imu_updated) {
		p_rad = sensor_combined.gyro_rad[0];
		q_rad = sensor_combined.gyro_rad[1];
		r_rad = sensor_combined.gyro_rad[2];
		ax_ms = sensor_combined.accelerometer_m_s2[0]*0.101972f;
		ay_ms = sensor_combined.accelerometer_m_s2[1]*0.101972f;
		az_ms = sensor_combined.accelerometer_m_s2[2]*0.101972f;
		if(!PX4_ISFINITE(p_rad) || !PX4_ISFINITE(q_rad) || !PX4_ISFINITE(r_rad) ){
			p_rad = 0.0f;
			q_rad = 0.0f;
			r_rad = 0.0f;
		}
		if(!PX4_ISFINITE(ax_ms) || !PX4_ISFINITE(ay_ms) || !PX4_ISFINITE(az_ms)){
			ax_ms = 0.0f;
			ay_ms = 0.0f;
			az_ms = -1.0f;
		}
		// PX4_INFO("%lf %lf %lf",(double)ax_ms,(double)ay_ms,(double)az_ms);
		accel_3dm.field_descriptor = 0x04;
		accel_3dm.field_length = 0x0e;
		accel_3dm.AccX = convertEndianFloat(&ax_ms);
		accel_3dm.AccY = convertEndianFloat(&ay_ms);
		accel_3dm.AccZ = convertEndianFloat(&az_ms);
		gyro_3dm.field_descriptor = 0x05;
		gyro_3dm.field_length = 0x0e;
		gyro_3dm.P = convertEndianFloat(&p_rad);
		gyro_3dm.Q = convertEndianFloat(&q_rad);
		gyro_3dm.R = convertEndianFloat(&r_rad);
	}
	vehicle_attitude_s vehicle_attitude;
	bool att_updated = false;
	att_updated = _vehicle_attitude_sub.update(&vehicle_attitude);
	if(att_updated){
		q0 = vehicle_attitude.q[0];
		q1 = vehicle_attitude.q[1];
		q2 = vehicle_attitude.q[2];
		q3 = vehicle_attitude.q[3];
		if(!PX4_ISFINITE(q0) || !PX4_ISFINITE(q1) ||!PX4_ISFINITE(q2) ||!PX4_ISFINITE(q3) ){
			q0 = 1.0f;
			q1 = 0.0f;
			q2 = 0.0f;
			q3 = 0.0f;
		}

		C11 = q0*q0+q1*q1-q2*q2-q3*q3;		C12 = 2.0f*q1*q2+2.0f*q0*q3;		C13 = 2.0f*q1*q3-2.0f*q0*q2;
		C21 = 2.0f*q1*q2-2.0f*q0*q3;		C22 = q0*q0-q1*q1+q2*q2-q3*q3;		C23 = 2.0f*q2*q3+2.0f*q0*q1;
		C31 = 2.0f*q1*q3+2.0f*q0*q2;		C32 = 2.0f*q2*q3-2.0f*q0*q1;		C33 = q0*q0-q1*q1-q2*q2+q3*q3;
		// PX4_INFO("%lf %lf %lf %lf",(double)q0,(double)q1,(double)q2,(double)q3);
		dcm_3dm.field_descriptor = 0x09;
		dcm_3dm.field_length = 0x26;
		dcm_3dm.C11 = convertEndianFloat(&C11);
		dcm_3dm.C12 = convertEndianFloat(&C12);
		dcm_3dm.C13 = convertEndianFloat(&C13);
		dcm_3dm.C21 = convertEndianFloat(&C21);
		dcm_3dm.C22 = convertEndianFloat(&C22);
		dcm_3dm.C23 = convertEndianFloat(&C23);
		dcm_3dm.C31 = convertEndianFloat(&C31);
		dcm_3dm.C32 = convertEndianFloat(&C32);
		dcm_3dm.C33 = convertEndianFloat(&C33);
	}
	vehicle_gps_position_s gps;
	if(_vehicle_gps_position_sub.update(&gps)){
		lat_int = gps.lat;
		lon_int = gps.lon;
		lat = (double)(lat_int)*0.0000001;
		lon = (double)(lat_int)*0.0000001;
		alt = (float)(gps.alt)*0.001f;
		alt_ellipsoid = (float)(gps.alt_ellipsoid)*0.001f;
		llh_3dm.field_descriptor = 0x03;
		llh_3dm.field_length = 0x2c;
		llh_3dm.latitude = convertEndianDouble(&lat);
		llh_3dm.longitude = convertEndianDouble(&lon);
		double temp = (double)alt;
		llh_3dm.height_above_msl = convertEndianDouble(&temp);
		temp = (double)gps.alt_ellipsoid;
		llh_3dm.height_above_ellipsoid = convertEndianDouble(&temp);
		llh_3dm.horizontal_accuracy = convertEndianFloat(&gps.eph);
		llh_3dm.vertical_accuracy = convertEndianFloat(&gps.epv);

		vN = gps.vel_n_m_s;
		vE = gps.vel_e_m_s;
		vD = gps.vel_d_m_s;
		vned_3dm.field_descriptor = 0x05;
		vned_3dm.field_length = 0x24;
		vned_3dm.v_north = convertEndianFloat(&vN);
		vned_3dm.v_east = convertEndianFloat(&vE);
		vned_3dm.v_down = convertEndianFloat(&vD);
		float tempf = sqrtf(gps.vel_n_m_s*gps.vel_n_m_s + gps.vel_e_m_s*gps.vel_e_m_s + gps.vel_d_m_s*gps.vel_d_m_s);
		vned_3dm.speed = convertEndianFloat(&tempf);
		tempf = sqrtf(gps.vel_n_m_s*gps.vel_n_m_s + gps.vel_e_m_s*gps.vel_e_m_s );
		vned_3dm.ground_speed = convertEndianFloat(&tempf);
		tempf = atan2f(gps.vel_e_m_s, gps.vel_n_m_s) * 57.2958f;
		vned_3dm.heading = convertEndianFloat(&tempf);
		tempf = 0.0f;
		vned_3dm.heading_accuracy = convertEndianFloat(&tempf);
		if(gps.fix_type>2){
			llh_3dm.valid = 0x1f00;
			vned_3dm.valid = 0x3f00;
		}else{
			llh_3dm.valid = 0x0000;
			vned_3dm.valid = 0x0000;
		}

	}
	// MICROSTRAIN_POLL_AHRS ahrs_packet;
	// Make_Poll_AHRS(&ahrs_packet, &accel_3dm, &gyro_3dm, &dcm_3dm);
	// MICROSTRAIN_POLL_GPS gps_packet;
	// Make_Poll_GPS(&gps_packet, &llh_3dm, &vned_3dm);

	//
	//unsigned char tbuf[512] = {0,};
	int len = ::read(serial_fd,&rbuf,128);

	// if(len>0){
	// 	PX4_INFO("received %d",len);
	// }
	AHRS_Protocol_Parsing(rbuf, len);
	if ((reply_command & REPLY_PING) == REPLY_PING)
	{
		reply_command = reply_command^REPLY_PING;
		::write(serial_fd,reply_ping,10);
		//cout << "reply" << endl;
	}
	if ((reply_command & REPLY_STREAM) == REPLY_STREAM)
	{
		MICROSTRAIN_DISABLE_STREAM stream_packet;
		reply_command = reply_command^REPLY_STREAM;
		Make_Stream_Reply(&stream_packet);
		::write(serial_fd,(uint8_t*)&stream_packet, sizeof(MICROSTRAIN_DISABLE_STREAM));
	}
	if ((reply_command & REPLY_SET_AHRS) == REPLY_SET_AHRS)
	{
		MICROSTRAIN_SET_FORMAT set_format_packet;
		reply_command = reply_command^REPLY_SET_AHRS;
		Make_Set_Format(&set_format_packet, IMU_MESSAGE_FORMAT_SET::IMU_MESSAGE_FORMAT);
		::write(serial_fd,(uint8_t*)&set_format_packet, sizeof(MICROSTRAIN_SET_FORMAT));
	}
	if ((reply_command & REPLY_SET_GPS) == REPLY_SET_GPS)
	{
		MICROSTRAIN_SET_FORMAT set_format_packet;
		reply_command = reply_command^REPLY_SET_GPS;
		Make_Set_Format(&set_format_packet, IMU_MESSAGE_FORMAT_SET::GPS_MESSAGE_FORMAT);
		::write(serial_fd,(uint8_t*)&set_format_packet, sizeof(MICROSTRAIN_SET_FORMAT));
	}
	if ((reply_command & REPLY_POLL_AHRS) == REPLY_POLL_AHRS)
	{
		MICROSTRAIN_POLL_AHRS poll_ahrs;
		reply_command = reply_command^REPLY_POLL_AHRS;
		//EnterCriticalSection(&crit);
		Make_Poll_AHRS(&poll_ahrs, &accel_3dm, &gyro_3dm, &dcm_3dm);
		::write(serial_fd,(uint8_t*)&poll_ahrs, sizeof(MICROSTRAIN_POLL_AHRS));

		//LeaveCriticalSection(&crit);
	}
	if ((reply_command & REPLY_POLL_GPS) == REPLY_POLL_GPS)
	{
		MICROSTRAIN_POLL_GPS poll_gps;
		reply_command = reply_command^REPLY_POLL_GPS;
		//EnterCriticalSection(&crit);
		Make_Poll_GPS(&poll_gps, &llh_3dm, &vned_3dm);
		::write(serial_fd,(uint8_t*)&poll_gps, sizeof(MICROSTRAIN_POLL_GPS));
		//LeaveCriticalSection(&crit);
	}
	perf_end(_loop_perf);
}


int MicroStrainAhrs::task_spawn(int argc, char *argv[])
{

	MicroStrainAhrs *instance = new MicroStrainAhrs();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			PX4_INFO("hello");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MicroStrainAhrs::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MicroStrainAhrs::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the microstrain 3DM-GX3-35 mimic

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("microstrain_ahrs", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int microstrain_ahrs_main(int argc, char *argv[])
{
	return MicroStrainAhrs::main(argc, argv);
}

