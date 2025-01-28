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
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

MicroStrainAhrs::MicroStrainAhrs() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::INS0),
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

	serial_fd = open("/dev/ttyS6", O_RDWR | O_NOCTTY | O_NONBLOCK);
	tcflush(serial_fd, TCIOFLUSH);
	int speed = 230400;

	if (serial_fd < 0) {
		warnx("failed to open port: /dev/ttyS6");
		return -1;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
/* Initialize the uart config */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		warnx("ERR GET CONF /dev/ttyS6");
		::close(serial_fd);
		return -1;
	}
/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR SET BAUD /dev/ttyS6: %d\n", termios_state);
		::close(serial_fd);
		return -1;
	}
	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}
	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR SET CONF /dev/ttyS6\n");
		::close(serial_fd);
		return -1;
	}

	uart_config.c_cflag |= CRTSCTS;
	tcsetattr(serial_fd, TCSANOW, &uart_config);
	PX4_INFO("serial port open at /dev/ttyS6(GPS2port)");
	return true;
}

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
		ax_ms = sensor_combined.accelerometer_m_s2[0];
		ay_ms = sensor_combined.accelerometer_m_s2[1];
		az_ms = sensor_combined.accelerometer_m_s2[2];
		// PX4_INFO("%lf %lf %lf",(double)ax_ms,(double)ay_ms,(double)az_ms);
		accel_3dm.field_descriptor = 0x04;
		accel_3dm.field_length = 0x0e;
		accel_3dm.AccX = ax_ms;
		accel_3dm.AccY = ay_ms;
		accel_3dm.AccZ = az_ms;
		gyro_3dm.field_descriptor = 0x05;
		gyro_3dm.field_length = 0x0e;
		gyro_3dm.P = p_rad;
		gyro_3dm.Q = q_rad;
		gyro_3dm.R = r_rad;
	}
	vehicle_attitude_s vehicle_attitude;
	bool att_updated = false;
	att_updated = _vehicle_attitude_sub.update(&vehicle_attitude);
	if(att_updated){
		q0 = vehicle_attitude.q[0];
		q1 = vehicle_attitude.q[1];
		q2 = vehicle_attitude.q[2];
		q3 = vehicle_attitude.q[3];

		C11 = q0*q0+q1*q1-q2*q2-q3*q3;		C12 = 2.0f*q1*q2+2.0f*q0*q3;		C13 = 2.0f*q1*q3-2.0f*q0*q2;
		C21 = 2.0f*q1*q2-2.0f*q0*q3;		C22 = q0*q0-q1*q1+q2*q2-q3*q3;		C23 = 2.0f*q2*q3+2.0f*q0*q1;
		C31 = 2.0f*q1*q3+2.0f*q0*q2;		C32 = 2.0f*q2*q3-2.0f*q0*q1;		C33 = q0*q0-q1*q1-q2*q2+q3*q3;
		// PX4_INFO("%lf %lf %lf %lf",(double)q0,(double)q1,(double)q2,(double)q3);
		dcm_3dm.field_descriptor = 0x09;
		dcm_3dm.field_length = 0x26;
		dcm_3dm.C11 = C11;
		dcm_3dm.C12 = C12;
		dcm_3dm.C13 = C13;
		dcm_3dm.C21 = C21;
		dcm_3dm.C22 = C22;
		dcm_3dm.C23 = C23;
		dcm_3dm.C31 = C31;
		dcm_3dm.C32 = C32;
		dcm_3dm.C33 = C33;
	}
	vehicle_global_position_s gpos;
	vehicle_local_position_s lpos;
	vehicle_gps_position_s gps;
	if(_vehicle_gps_position_sub.update(&gps)){
		lat_int = gps.lat;
		lon_int = gps.lon;
		llh_3dm.field_descriptor = 0x03;
		llh_3dm.field_length = 0x2c;
	}
	if (_vehicle_global_position_sub.update(&gpos)){
		lat = gpos.lat;
		lon = gpos.lon;
		alt = gpos.alt;
		llh_3dm.field_descriptor = 0x03;
		llh_3dm.field_length = 0x2c;
		llh_3dm.latitude = lat;
		llh_3dm.longitude = lon;
		llh_3dm.height_above_msl = (double)alt;
		llh_3dm.height_above_ellipsoid = (double)gpos.alt_ellipsoid;
		llh_3dm.horizontal_accuracy = gpos.eph;
		llh_3dm.vertical_accuracy = gpos.epv;
		llh_3dm.valid = 0x1f00;
	}
	if(_vehicle_local_position_sub.update(&lpos))
	{
		vN = lpos.vx;
		vE = lpos.vy;
		vD = lpos.vz;
		vned_3dm.field_descriptor = 0x05;
		vned_3dm.field_length = 0x24;
		vned_3dm.v_north = vN;
		vned_3dm.v_east = vE;
		vned_3dm.v_down = vD;
		vned_3dm.speed = sqrtf(vN*vN + vE*vE + vD*vD);
		vned_3dm.ground_speed = sqrtf(vN*vN + vE*vE);
		vned_3dm.heading = atan2f(vE, vN) * 57.2958f;
		vned_3dm.heading_accuracy = 0.0;
		vned_3dm.valid = 0x3f00;
	}

	// MICROSTRAIN_POLL_AHRS ahrs_packet;
	// Make_Poll_AHRS(&ahrs_packet, &accel_3dm, &gyro_3dm, &dcm_3dm);
	// MICROSTRAIN_POLL_GPS gps_packet;
	// Make_Poll_GPS(&gps_packet, &llh_3dm, &vned_3dm);

	int len = ::read(serial_fd,&rbuf,256);
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

