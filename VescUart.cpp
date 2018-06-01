/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.
*/

#include <SerialPort.h>

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <unistd.h>

#include "VescUart.h"
#include "buffer.h"
#include "crc.h"

using namespace LibSerial;

// static HardwareSerial* serialPort1;
// static HardwareSerial* serialPort2;
// static HardwareSerial* serialPort3;
// static HardwareSerial* serialPort4;
// static HardwareSerial* debugSerialPort = NULL;

// Instantiate a SerialPort object.
static SerialPort serial_port;

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);
bool ProcessReadPacket(uint8_t* message, struct bldcMeasure& values, int len);

// void SetSerialPort(HardwareSerial*  _serialPort1, HardwareSerial*  _serialPort2, HardwareSerial*  _serialPort3, HardwareSerial*  _serialPort4)
// {
// 	serialPort1 = _serialPort1;
// 	serialPort2 = _serialPort2;
// 	serialPort3 = _serialPort3;
// 	serialPort4 = _serialPort4;
// }

// void SetSerialPort(HardwareSerial* _serialPort)
// {
// 	SetSerialPort(_serialPort, _serialPort, _serialPort, _serialPort);
// }

// void SetDebugSerialPort(HardwareSerial * _debugSerialPort)
// {
// 	debugSerialPort = _debugSerialPort;
// }

void SerialPortInit(const char *name) {
    // Open the Serial Port at the desired hardware port.
    // serial_port.Open("/dev/ttyUSB0");
    serial_port.Open(name);

    // Set the baud rate of the serial port.
    serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE);

    // Set the number of stop bits.
    serial_port.SetStopBits(StopBits::STOP_BITS_1);
}

//HardwareSerial *serial; ///@param num as integer with the serial port in use (0=Serial; 1=Serial1; 2=Serial2; 3=Serial3;)
int ReceiveUartMessage(uint8_t* payloadReceived, int num) {

	//Messages <= 255 start with 2. 2nd byte is length
	//Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	int counter = 0;
	int endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	int lenPayload = 0;
	// HardwareSerial* serial;


	// switch (num) {
	// 	case 0:
	// 		serial = serialPort1;
	// 		break;
	// 	case 1:
	// 		serial = serialPort2;
	// 		break;
	// 	case 2:
	// 		serial = serialPort3;
	// 		break;
	// 	case 3:
	// 		serial = serialPort4;
	// 		break;
	// 	default:
	// 		break;

	// }

  // Timeout value in milliseconds to wait for data being read.
  // size_t ms_timeout = 250;
  size_t ms_timeout = 0;

  // Char variable to store data coming from the serial port.
  char data_byte;

  // Wait for data to be available at the serial port.
	while (serial_port.IsDataAvailable()) {

    // Read one byte from the serial port.
    try {
        // Read a single byte of data from the serial port.
        serial_port.ReadByte(data_byte, ms_timeout);
    } catch (ReadTimeout) {
        std::cerr << "\nThe ReadByte() call has timed out." << std::endl;
    }

    // Read one byte from the serial port.
		messageReceived[counter++] = data_byte;

		if (counter == 2) {//case if state of 'counter' with last read 1

			switch (messageReceived[0])
			{
			case 2:
				endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
				lenPayload = messageReceived[1];
				break;
			case 3:
				//ToDo: Add Message Handling > 255 (starting with 3)
				break;
			default:
				break;
			}

		}
		if (counter >= sizeof(messageReceived))
		{
			break;
		}

		if (counter == endMessage && messageReceived[endMessage - 1] == 3) {//+1: Because of counter++ state of 'counter' with last read = "endMessage"
			messageReceived[endMessage] = 0;
      printf("End of message reached!\n");
			messageRead = true;
			break; //Exit if end of message is reached, even if there is still more data in buffer.
		}
	}
	bool unpacked = false;
	if (messageRead) {
		unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
	}
	if (unpacked)
	{
		return lenPayload; //Message was read

	}
	else {
		return 0; //No Message Read
	}
}

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay) {
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;
	//Rebuild src:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];
	printf("SRC received: %02x ", crcMessage);

	//Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);
	printf("SRC calc: %02x ", crcPayload);
	if (crcPayload == crcMessage)
	{
		printf("Received: "); SerialPrint(message, lenMes); printf("\n");
		printf("Payload :      "); SerialPrint(payload, message[1] - 1); printf("\n");

		return true;
	}
	else
	{
		return false;
	}
}




int PackSendPayload(uint8_t* payload, int lenPay, int num) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = NULL;

	printf("UART package send: "); SerialPrint(messageSend, count);

	// HardwareSerial *serial;

	// switch (num) {
	// 	case 0:
	// 		serial=serialPort1;
	// 		break;
	// 	case 1:
	// 		serial=serialPort2;
	// 		break;
	// 	case 2:
	// 		serial=serialPort3;
	// 		break;
	// 	case 3:
	// 		serial=serialPort4;
	// 		break;
	// 	default:
	// 		break;
	// }

	//Sending package
	// serial->write(messageSend, count);

  DataBuffer write_buffer;
  for(int i = 0; i < count; ++i) {
    // write_buffer[i] = messageSend[i];
    write_buffer.push_back(messageSend[i]);
  }

  // Write the data to the serial port.
  serial_port.Write(write_buffer);

  // Wait until the data has actually been transmitted.
  serial_port.DrainWriteBuffer();


	//Returns number of send bytes
	return count;
}


bool ProcessReadPacket(uint8_t* message, struct bldcMeasure& values, int len) {
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{

	case COMM_GET_VALUES:
		values.tempFetFiltered		= buffer_get_float16(message, 1e1, &ind);
		values.tempMotorFiltered	= buffer_get_float16(message, 1e1, &ind);
		values.avgMotorCurrent		= buffer_get_float32(message, 100.0, &ind);
		values.avgInputCurrent		= buffer_get_float32(message, 100.0, &ind);
		values.avgId				= buffer_get_float32(message, 1e2, &ind);
		values.avgIq				= buffer_get_float32(message, 1e2, &ind);
		values.dutyNow				= buffer_get_float16(message, 1000.0, &ind);
		values.rpm					= buffer_get_float32(message, 1.0, &ind);
		values.inpVoltage			= buffer_get_float16(message, 10.0, &ind);
		values.ampHours				= buffer_get_float32(message, 10000.0, &ind);
		values.ampHoursCharged		= buffer_get_float32(message, 10000.0, &ind);
		values.wattHours			= buffer_get_float32(message, 1e4, &ind);
		values.watthoursCharged		= buffer_get_float32(message, 1e4, &ind);
		values.tachometer			= buffer_get_int32(message, &ind);
		values.tachometerAbs		= buffer_get_int32(message, &ind);
		values.faultCode			= message[ind];
		return true;
		break;
	default:
		return false;
		break;
	}

}

bool VescUartGetValue(bldcMeasure& values, int num) {
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];
	PackSendPayload(command, 1, num);
	usleep(10); //needed, otherwise data is not read
	int lenPayload = ReceiveUartMessage(payload, num);
	if (lenPayload > 1) {
		bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}
bool VescUartGetValue(bldcMeasure& values) {
	return VescUartGetValue(values, 0);
}

void VescUartSetCurrent(float current, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	PackSendPayload(payload, 5, num);
}
void VescUartSetCurrent(float current){
	VescUartSetCurrent(current, 0);
}

void VescUartSetPosition(float position, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_POS ;
	buffer_append_int32(payload, (int32_t)(position * 1000000.0), &index);
	PackSendPayload(payload, 5, num);
}

void VescUartSetPosition(float position) {
	VescUartSetPosition(position, 0);
}

void VescUartSetServoPosition(float position, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_SERVO_POS ;
	buffer_append_int32(payload, (int32_t)(position * 1000), &index);
	PackSendPayload(payload, 5, num);
}

void VescUartSetServoPosition(float position) {
  VescUartSetServoPosition(position, 0);
}

void VescUartSetDuty(float duty, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY ;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);
	PackSendPayload(payload, 5, num);
}
void VescUartSetDuty(float duty) {
	VescUartSetDuty(duty, 0);
}


void VescUartSetRPM(float rpm, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);
	PackSendPayload(payload, 5, num);
}
void VescUartSetRPM(float rpm) {
	VescUartSetRPM(rpm, 0);
}

void VescUartSetCurrentBrake(float brakeCurrent, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	PackSendPayload(payload, 5, num);
}
void VescUartSetCurrentBrake(float brakeCurrent) {
	VescUartSetCurrentBrake(brakeCurrent, 0);
}


void VescUartSetNunchukValues(remotePackage& data, int num) {
	int32_t ind = 0;
	uint8_t payload[11];
	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = data.valXJoy;
	payload[ind++] = data.valYJoy;
	buffer_append_bool(payload, data.valLowerButton, &ind);
	buffer_append_bool(payload, data.valUpperButton, &ind);
	//Acceleration Data. Not used, Int16 (2 byte)
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;

	printf("Data reached at VescUartSetNunchuckValues:\n");
	printf("valXJoy = %d valYJoy = %d\n", data.valXJoy, data.valYJoy);
	printf("LowerButton = %d UpperButton = %d\n", data.valLowerButton, data.valUpperButton);

	PackSendPayload(payload, 11, num);
}
void VescUartSetNunchukValues(remotePackage& data) {
	VescUartSetNunchukValues(data, 0);
}


void SerialPrint(uint8_t* data, int len) {

	//	debugSerialPort->print("Data to display: "); debugSerialPort->println(sizeof(data));

	for (int i = 0; i <= len; i++)
	{
		printf("%02x ", data[i]);
	}
	printf("\n");
}


void SerialPrint(const struct bldcMeasure& values) {
	printf("tempFetFiltered:	%f\n", values.tempFetFiltered);
	printf("tempMotorFiltered:%f\n", values.tempMotorFiltered);
	printf("avgMotorCurrent:	%f\n", values.avgMotorCurrent);
	printf("avgInputCurrent:	%f\n", values.avgInputCurrent);
	printf("avgId:			%f\n", values.avgId);
	printf("avgIq:			%f\n", values.avgIq);
	printf("dutyNow:			%f\n", values.dutyNow);
	printf("rpm:				%ld\n", values.rpm);
	printf("inpVoltage:		%f\n", values.inpVoltage);
	printf("ampHours:		%f\n", values.ampHours);
	printf("ampHoursCharged:	%f\n", values.ampHoursCharged);
	printf("tachometer:		%ld\n", values.tachometer);
	printf("tachometerAbs:	%ld\n", values.tachometerAbs);
	printf("faultCode:		%d\n", values.faultCode);
}
