#pragma once

#include <MicroStrain.hpp>

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

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>



using namespace time_literals;
class MicroStrainAhrs  : public ModuleBase<MicroStrainAhrs>, public ModuleParams, public px4::ScheduledWorkItem, public MicroStrain
{
public:
	MicroStrainAhrs();
	~MicroStrainAhrs() override;
		/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();
	int serial_fd=-1;
private:
	perf_counter_t	_loop_perf;			/**< loop duration performance counter */
	void Run() override;
	void		parameters_updated();
	bool _callback_registered{false};
	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	unsigned char reply_ping[10] = { 0x75, 0x65, 0x01, 0x04, 0x04, 0xF1, 0x01, 0x00, 0xd5, 0x6a };
	float p_rad=0.0f;
	float q_rad=0.0f;
	float r_rad=0.0f;
	float ax_ms=0.0f;
	float ay_ms=0.0f;
	float az_ms=-9.80665f;
	float q0 = 1.0f;
	float q1 = 0.0f;
	float q2 = 0.0f;
	float q3 = 0.0f;
	float vN = 0.0f;
	float vE = 0.0f;
	float vD = 0.0f;
	double lat = 0.0f;
	double lon = 0.0f;
	float alt = 0.0f;
	int lat_int = 0;
	int lon_int = 0;

	float C11 = 0.0f;
	float C12 = 0.0f;
	float C13 = 0.0f;
	float C21 = 0.0f;
	float C22 = 0.0f;
	float C23 = 0.0f;
	float C31 = 0.0f;
	float C32 = 0.0f;
	float C33 = 0.0f;

	ACCEL_3DM accel_3dm;
	GYRO_3DM gyro_3dm;
	DCM_3DM dcm_3dm;
	LLH_3DM llh_3dm;
	VNED_3DM vned_3dm;
};
