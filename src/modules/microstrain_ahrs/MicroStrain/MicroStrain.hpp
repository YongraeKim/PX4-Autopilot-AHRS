#pragma once

/*****************************************************************************
*							HILS using Microstrain AHRS
******************************************************************************/
#define IMU_BASE_COMMAND_SET			0x01
#define IMU_SETTING_COMMAND_SET			0x0C
#define IMU_IMU_DATA_PROTOCOL			0x80
#define IMU_GPS_DATA_PROTOCOL			0x81
#define IMU_HEADER_FOUND				0x01
#define IMU_FOUND_PROTOCOL_DISCRIPT		0x02
#define IMU_FOUND_PROTOCOL_LENGTH		0x03
#define IMU_FOUND_PROTOCOL_TOTAL(x)		x+0x06

typedef enum
{
	POLL_AHRS_DATA = 0x01,
	POLL_GPS_DATA = 0x02
}POLL_IMU_GPS_DATA;

typedef enum
{
	PING_COMM_FIELD_LENGTH = 0x02,
	PING_COMM_DISCRIPT = 0x01,
	PING_REPLY_FIELD_LENGTH = 0x04,
	PING_REPLY_DISCRIPT = 0xF1
}IMU_PING_COMM_REPLY;

typedef enum
{
	DATA_STREAM_DEVICE_AHRS = 0x01,
	DATA_STREAM_DEVICE_GPS = 0x02
}IMU_DATA_STREAM_DEVICE;

typedef enum
{
	DATA_STREAM_COMMAND_FIELD_DESCRIPT = 0x11,
	DATA_STREAM_COMMAND_ECHO_DISCRIPT = 0x11,
	DATA_STREAM_COMMAND_ERROR_CODE = 0x00
}IMU_DATA_STREAM_COMMAND_ECHO_SET;

typedef enum
{
	DATA_STREAM_NEW_SET = 0x01,
	DATA_STREAM_READ_SET = 0x02,
	DATA_STREAM_SAVE_SET = 0x03,
	DATA_STREAM_LOAD_SET = 0x04,
	DATA_STREAM_DEFAULT_SET = 0x05
}IMU_DATA_STREAM_FUNCTION;

typedef enum
{
	DATA_STREAM_DISABLE_STREAM = 0x00,
	DATA_STREAM_ENABLE_STREAM = 0x01
}IMU_DATA_STREAM_STATE;

typedef enum
{
	IMU_DATA_STREAM_COMMAND_DISCRIPT = 0x08,
	IMU_DATA_STREAM_REPLY_LENGTH = 0x04,
	IMU_DATA_STREAM_REPLY_DISCRIPT_1 = 0xF1,
	IMU_DATA_STREAM_REPLY_DISCRIPT_2 = 0x80		//use this if DATA_STREAM_READ_SET
}IMU_IMU_DATA_STREAM_REPLY;

typedef enum
{
	GPS_DATA_STREAM_COMMAND_LENGTH = 0x09,
	GPS_DATA_STREAM_REPLY_LENGTH = 0x04,
	GPS_DATA_STREAM_REPLY_DISCRIPT_1 = 0xF1,
	GPS_DATA_STREAM_REPLY_DISCRIPT_2 = 0x81		//use this if DATA_STREAM_READ_SET
}IMU_GPS_DATA_STREAM_REPLY;

typedef enum
{
	IMU_MESSAGE_FORMAT = 0x08,
	GPS_MESSAGE_FORMAT = 0x09
}IMU_MESSAGE_FORMAT_SET;


typedef enum
{
	GPS_DATA_POSITION_LLH_LENGTH = 0x2C,
	GPS_DATA_POSITION_LLH_DISCRIPT = 0x03,
	GPS_DATA_VELOCITY_NED_LENGTH = 0x24,
	GPS_DATA_VELOCITY_NED_DISCRIPT = 0x05
}IMU_GPS_DATA_SET;

typedef enum
{
	GPS_LATITUDE_LONGITUDE_VALID = 0x0001,
	GPS_ELLIPSOID_HEIGHT_VALID = 0x0002,
	GPS_MSL_HEIGHT_VALID = 0x0004,
	GPS_HORIZONTAL_ACCURACY_VALID = 0x0008,
	GPS_VERTICAL_ACCURACY_VALID = 0x0010
}GPS_LLH_VALID_FLAG_MAPPING;

typedef enum
{
	GPS_NED_VELOCITY_VALID = 0x0001,
	GPS_SPEED_VALID = 0x0002,
	GPS_GS_VALID = 0x0004,
	GPS_HEADING_VALID = 0x0008,
	GPS_SPEED_ACCURACY_VALID = 0x0010,
	GPS_HEADING_ACCURACY_VALID = 0x0020
}GPS_NED_VELOCITY_VALID_FLAG_MAPPING;



typedef enum
{
	REPLY_EMPTY = 0x00,
	REPLY_PING = 0x01,
	REPLY_STREAM = 0x02,
	REPLY_SET_AHRS = 0x04,
	REPLY_SET_GPS = 0x08,
	REPLY_POLL_AHRS = 0x10,
	REPLY_POLL_GPS = 0x20

}REPLY_COMMAND;

typedef enum
{
	IMU_PROTOCOL_CHECKSUM_MATCHED = 0x01,
	IMU_PROTOCOL_CHECKSUM_MISMATCHED = 0x00
}IMU_PROTOCOL_CHECKSUM;

typedef enum
{
	ENCODER_PROTOCOL_CHECKSUM_MATCH = 0x01,
	ENCODER_PROTOCOL_CHECKSUM_MISMATCH = 0x00
}ENCODER_PROTOCOL_CHECKSUM;
#pragma pack(1)
typedef struct
{
	unsigned char sync1;
	unsigned char sync2;
	unsigned char descriptor;
	unsigned char payload_length;
	/* data */
}HEADER_3DM;

typedef struct
{
	unsigned char field_length;
	unsigned char field_descriptor;
	float AccX;
	float AccY;
	float AccZ;
	/* data */
}ACCEL_3DM;

typedef struct
{
	unsigned char field_length;
	unsigned char field_descriptor;
	float P;
	float Q;
	float R;
	/* data */
}GYRO_3DM;

typedef struct
{
	unsigned char field_length;
	unsigned char field_descriptor;
	float C11;
	float C12;
	float C13;
	float C21;
	float C22;
	float C23;
	float C31;
	float C32;
	float C33;
}DCM_3DM;

typedef struct
{
	unsigned char field_length;
	unsigned char field_descriptor;
	float q0;
	float q1;
	float q2;
	float q3;
	/* data */
}QUAT_3DM;

typedef struct
{
	unsigned char field_length;
	unsigned char field_descriptor;
	double latitude;
	double longitude;
	double height_above_ellipsoid;
	double height_above_msl;
	float horizontal_accuracy;
	float vertical_accuracy;
	unsigned short valid;
	/* data */
}LLH_3DM;

typedef struct
{
	unsigned char field_length;
	unsigned char field_descriptor;
	float v_north;
	float v_east;
	float v_down;
	float speed;
	float ground_speed;
	float heading;
	float speed_accuracy;
	float heading_accuracy;
	unsigned short valid;
	/* data */
}VNED_3DM;
typedef struct
{
	unsigned char Buffer[128];
	int index;
	int Payload_Length;
	int Total_Length;
	int Field_Length;
	int Field_Start;
}PROTOCOL_BUFFER;


typedef struct
{
	uint8_t sync1;
	uint8_t sync2;
	uint8_t descriptor_set;
	uint8_t payload_length;
}MICROSTRAIN_AHRS_HEADER;

typedef struct
{
	uint8_t field_length;
	uint8_t field_descriptor;
}MICROSTRAIN_FIELD_DESCRIPTOR;


typedef struct
{
	uint8_t msb;
	uint8_t lsb;
}MICROSTRAIN_AHRS_CHECKSUM;

typedef struct
{
	uint8_t field_length;
	uint8_t field_descriptor;
	uint64_t field_data;
}MICROSTRAIN_STREAM_REPLY;

typedef struct
{
	uint8_t field_length;
	uint8_t field_descriptor;
	uint16_t field_data;
}MICROSTRAIN_SET_FORMAT_REPLY;


typedef struct
{
	MICROSTRAIN_AHRS_HEADER header;
	MICROSTRAIN_STREAM_REPLY setting;
	MICROSTRAIN_AHRS_CHECKSUM checksum;
}MICROSTRAIN_DISABLE_STREAM;

typedef struct
{
	MICROSTRAIN_AHRS_HEADER header;
	MICROSTRAIN_SET_FORMAT_REPLY setting;
	MICROSTRAIN_AHRS_CHECKSUM checksum;
}MICROSTRAIN_SET_FORMAT;

typedef struct
{
	MICROSTRAIN_AHRS_HEADER header;
	ACCEL_3DM accel;
	GYRO_3DM gyro;
	DCM_3DM dcm;
	MICROSTRAIN_AHRS_CHECKSUM checksum;
}MICROSTRAIN_POLL_AHRS;

typedef struct
{
	MICROSTRAIN_AHRS_HEADER header;
	LLH_3DM llh;
	VNED_3DM vned;
	MICROSTRAIN_AHRS_CHECKSUM checksum;
}MICROSTRAIN_POLL_GPS;
#pragma pack()



class MicroStrain
{
public:
	MicroStrain(){};
	virtual ~MicroStrain(){};

protected:
	void AHRS_Protocol_Parsing(uint8_t* pBuff, int32_t length);
	uint8_t reply_command;
private:
	uint8_t Flag;
	uint16_t Sync;
	uint8_t Protocol_Discriptor;
	IMU_PROTOCOL_CHECKSUM Protocol_Checksum_Insepection();
	PROTOCOL_BUFFER Protocol_Parse_Buffer;


protected:
	void Make_Header(MICROSTRAIN_AHRS_HEADER* header, uint8_t descriptor, uint8_t payload_length);
	void Make_Stream_Reply(MICROSTRAIN_DISABLE_STREAM* stream_packet);
	void Make_Checksum(void* packet, uint8_t packet_length, MICROSTRAIN_AHRS_CHECKSUM* checksum);
	void Make_Set_Format(MICROSTRAIN_SET_FORMAT* set_format_packet, IMU_MESSAGE_FORMAT_SET ahrs_gps);
	void Make_Poll_AHRS(MICROSTRAIN_POLL_AHRS* poll_ahrs, ACCEL_3DM *accel, GYRO_3DM *gyro, DCM_3DM *dcm);
	void Make_Poll_GPS(MICROSTRAIN_POLL_GPS* poll_gps, LLH_3DM *llh, VNED_3DM *vned);
	ENCODER_PROTOCOL_CHECKSUM Protocol_Checksum_Inspection_8Bit(uint8_t* pbuff, int32_t length);
	ENCODER_PROTOCOL_CHECKSUM Protocol_Checksum_Inspection_16Bit(uint8_t* pbuff, int32_t length);
	float convertEndianFloat(float *target)
	{
		//unsigned long swap = (unsigned long)*target;
		//swap = _byteswap_ulong(swap);
		//*target = (float)swap;
		//return (float)swap;
		float after_conversion;
		unsigned char *medium = (unsigned char *)&after_conversion;
		unsigned char *conversion;
		conversion = (unsigned char*)target;
		for (unsigned int j = 0; j<(sizeof(float)); j++)
		{
			medium[j] = conversion[(sizeof(float)-1) - j];
		} //for: endian transformation
		*target = (float)after_conversion;
		return after_conversion;
	}

	double convertEndianDouble(double *target)
	{
		//UINT64 swap = (UINT64)(*target);
		//swap = _byteswap_uint64(swap);
		//*target = (double)(swap);
		//return (double)swap;
		double after_conversion;
		unsigned char *medium = (unsigned char *)&after_conversion;
		unsigned char *conversion;
		conversion = (unsigned char*)target;

		for (unsigned int j = 0; j<(sizeof(double)); j++)
		{
			medium[j] = conversion[(sizeof(double)-1) - j];
		} //for: endian transformation
		*target = (double)after_conversion;
		return after_conversion;
	}
private:
	int32_t aircraft_number;
protected:
	void Set_Aircraft_Number(int32_t aircraft_num);
};

