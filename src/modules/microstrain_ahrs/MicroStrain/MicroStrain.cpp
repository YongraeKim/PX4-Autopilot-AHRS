#include <MicroStrain.hpp>
#include <px4_platform_common/defines.h>
#include <string.h>
IMU_PROTOCOL_CHECKSUM MicroStrain::Protocol_Checksum_Insepection()
{
	unsigned char checksum_msb = 0, checksum_lsb = 0;
	int i = 0;
	for (i = 0; i < Protocol_Parse_Buffer.Payload_Length + 4; i++)
	{
		checksum_msb = checksum_msb + Protocol_Parse_Buffer.Buffer[i];
		checksum_lsb = checksum_lsb + checksum_msb;
	} //for: checksum generation for inspection
	if ((checksum_msb == Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.Payload_Length + 4]) &&
		(checksum_lsb == Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.Payload_Length + 5]))
	{
		return IMU_PROTOCOL_CHECKSUM_MATCHED;
	}
	else
	{
		return IMU_PROTOCOL_CHECKSUM_MISMATCHED;
	} //if: checksum inspection
}


void MicroStrain::AHRS_Protocol_Parsing(uint8_t* pBuff, int32_t length)
{
	for (int32_t i = 0; i<length; i++)
	{
		Sync |= pBuff[i];
		if (Flag == 1)
		{
			Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.index] = pBuff[i];
			if (Protocol_Parse_Buffer.index == IMU_FOUND_PROTOCOL_DISCRIPT)
			{
				Protocol_Discriptor = Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.index];
				Protocol_Parse_Buffer.index++;
			}
			else if (Protocol_Parse_Buffer.index == IMU_FOUND_PROTOCOL_LENGTH)
			{
				Protocol_Parse_Buffer.Payload_Length = Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.index];
				Protocol_Parse_Buffer.index++;
			}
			else if (Protocol_Parse_Buffer.index + 1 == IMU_FOUND_PROTOCOL_TOTAL(Protocol_Parse_Buffer.Payload_Length))
			{
				if (Protocol_Checksum_Insepection() == IMU_PROTOCOL_CHECKSUM_MATCHED)
				{
					switch (Protocol_Discriptor)
					{
					case IMU_BASE_COMMAND_SET:
						if ((Protocol_Parse_Buffer.Buffer[5] == PING_COMM_DISCRIPT) && (Protocol_Parse_Buffer.Buffer[4] == PING_COMM_FIELD_LENGTH))
						{
							reply_command |= REPLY_PING;
						}
						break;
					case IMU_SETTING_COMMAND_SET:
						Protocol_Parse_Buffer.Field_Start = 4;
						while (Protocol_Parse_Buffer.Payload_Length>0)
						{
							Protocol_Parse_Buffer.Field_Length = Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.Field_Start];
							Protocol_Parse_Buffer.Payload_Length = Protocol_Parse_Buffer.Payload_Length - Protocol_Parse_Buffer.Field_Length;
							switch (Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.Field_Start + 1])
							{
							case DATA_STREAM_COMMAND_FIELD_DESCRIPT:
								switch (Protocol_Parse_Buffer.Buffer[Protocol_Parse_Buffer.Field_Start + 3])
								{
								case DATA_STREAM_DEVICE_AHRS:
									reply_command |= REPLY_STREAM;
									break;
								case DATA_STREAM_DEVICE_GPS:
									reply_command |= REPLY_STREAM;
									break;
								}
								break;
							case IMU_MESSAGE_FORMAT:
								reply_command |= REPLY_SET_AHRS;
								break;
							case GPS_MESSAGE_FORMAT:
								reply_command |= REPLY_SET_GPS;
								break;
							case POLL_AHRS_DATA:
								reply_command |= REPLY_POLL_AHRS;
								break;
							case POLL_GPS_DATA:
								reply_command |= REPLY_POLL_GPS;
								break;
							default:
								break;
							}
						}
						break;

					default:
						break;
					}

				}
				Protocol_Parse_Buffer.index = 0;
				Protocol_Parse_Buffer.Payload_Length = 0;
				Flag = 0;
				Protocol_Discriptor = 0;
				memset(Protocol_Parse_Buffer.Buffer, 0, 128);
			}
			else
			{
				Protocol_Parse_Buffer.index++;
			}
		}
		if (Sync == 0x7565)
		{
			Flag = 1;
			Protocol_Parse_Buffer.Buffer[0] = 0x75;
			Protocol_Parse_Buffer.Buffer[1] = 0x65;
			Protocol_Parse_Buffer.index = 2;

		}
		Sync = Sync << 8;
	}
}


void MicroStrain::Make_Header(MICROSTRAIN_AHRS_HEADER* header, uint8_t descriptor, uint8_t payload_length)
{
	header->sync1 = 0x75;
	header->sync2 = 0x65;
	header->descriptor_set = descriptor;
	header->payload_length = payload_length;
}


void MicroStrain::Make_Checksum(void* packet, uint8_t packet_length, MICROSTRAIN_AHRS_CHECKSUM* checksum)
{
	uint8_t msb = 0;
	uint8_t lsb = 0;
	uint8_t* packet_pointer = (uint8_t*)packet;
	for (int i = 0; i < packet_length - 2; i++)
	{
		msb = msb + packet_pointer[i];
		lsb = lsb + msb;
	}
	checksum->msb = msb;
	checksum->lsb = lsb;
}


void MicroStrain::Make_Stream_Reply(MICROSTRAIN_DISABLE_STREAM* stream_packet)
{
	uint8_t reply_disable_stream[12] = { 0x75, 0x65, 0x0C, 0x08, 0x04, 0xF1, 0x11, 0x00, 0x04, 0xF1, 0x11, 0x00 };
	memcpy(stream_packet, reply_disable_stream, 12);
	Make_Checksum((void *)stream_packet, 14, &stream_packet->checksum);
}


void MicroStrain::Make_Set_Format(MICROSTRAIN_SET_FORMAT* set_format_packet, IMU_MESSAGE_FORMAT_SET ahrs_gps)
{
	uint8_t reply_set_AHRS[8] = { 0x75, 0x65, 0x0C, 0x04, 0x04, 0xF1, 0x08, 0x00 };
	uint8_t reply_set_GPS[8] = { 0x75, 0x65, 0x0C, 0x04, 0x04, 0xF1, 0x09, 0x00 };
	switch (ahrs_gps)
	{
	case IMU_MESSAGE_FORMAT:
		memcpy(set_format_packet, reply_set_AHRS, 8);
		break;
	case GPS_MESSAGE_FORMAT:
		memcpy(set_format_packet, reply_set_GPS, 8);
		break;
	}
	Make_Checksum((void *)set_format_packet, sizeof(MICROSTRAIN_SET_FORMAT), &set_format_packet->checksum);
}


void MicroStrain::Make_Poll_AHRS(MICROSTRAIN_POLL_AHRS* poll_ahrs,ACCEL_3DM *accel,GYRO_3DM *gyro, DCM_3DM *dcm)
{
	Make_Header(&poll_ahrs->header, 0x80, sizeof(MICROSTRAIN_POLL_AHRS)-sizeof(MICROSTRAIN_AHRS_HEADER) - sizeof(MICROSTRAIN_AHRS_CHECKSUM));
	poll_ahrs->accel = *accel;
	poll_ahrs->gyro = *gyro;
	poll_ahrs->dcm = *dcm;
	Make_Checksum((void *)poll_ahrs, sizeof(MICROSTRAIN_POLL_AHRS), &poll_ahrs->checksum);
}


void MicroStrain::Make_Poll_GPS(MICROSTRAIN_POLL_GPS* poll_gps, LLH_3DM *llh, VNED_3DM *vned)
{
	Make_Header(&poll_gps->header, 0x81, sizeof(MICROSTRAIN_POLL_GPS)-sizeof(MICROSTRAIN_AHRS_HEADER)-sizeof(MICROSTRAIN_AHRS_CHECKSUM));
	poll_gps->llh = *llh;
	poll_gps->vned = *vned;
	Make_Checksum((void *)poll_gps, sizeof(MICROSTRAIN_POLL_GPS), &poll_gps->checksum);
}

ENCODER_PROTOCOL_CHECKSUM MicroStrain::Protocol_Checksum_Inspection_8Bit(uint8_t* pbuff, int32_t length)
{
	uint8_t checksum = 0;
	for (int32_t i = 0; i < length-1; i++)
	{
		checksum = checksum + pbuff[i];
	}
	if (checksum == pbuff[length - 1])
	{
		return ENCODER_PROTOCOL_CHECKSUM::ENCODER_PROTOCOL_CHECKSUM_MATCH;
	}
	else
	{
		return ENCODER_PROTOCOL_CHECKSUM::ENCODER_PROTOCOL_CHECKSUM_MISMATCH;
	}
}

ENCODER_PROTOCOL_CHECKSUM MicroStrain::Protocol_Checksum_Inspection_16Bit(uint8_t* pbuff, int32_t length)
{
	uint8_t checksum_first = 0;
	uint8_t checksum_second = 0;
	for (int32_t i = 0; i < length - 2; i++)
	{
		checksum_first = checksum_first + pbuff[i];
		checksum_second = checksum_second + checksum_first;
	}

	if (checksum_first == pbuff[length - 2] && checksum_second == pbuff[length - 1])
	{
		return ENCODER_PROTOCOL_CHECKSUM::ENCODER_PROTOCOL_CHECKSUM_MATCH;
	}
	else
	{
		return ENCODER_PROTOCOL_CHECKSUM::ENCODER_PROTOCOL_CHECKSUM_MISMATCH;
	}
}
