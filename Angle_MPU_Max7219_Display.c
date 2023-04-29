/*
    + Connect to Max7219:
        - VCC   3.3V        17 phys
        - GND   0v          20
        - Din   MOSI        19
        - CS    CE0,CE1     24,26, using Pin 24
        - SCLK              23
    + Connect to MPU6050:
        - VCC   3.3V        1 Physical
        - GND   0v          6
        - SDA               3
        - SCL               5
        - INT               7
*/

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <wiringPiSPI.h>

/* Define for Max7219 --------------------------------------------------------------------------*/
#define spi0                0
#define Decode_Mode_Reg     0x09    //Decode-Mode Register Examples (Table 4 datasheet)
#define Digits              0x00    //No Decode for digits 7-0 
#define Intensity_Reg       0x0A    // Intensity Register Format(Table 7)
#define Intensity           0x09    // 19/32 Duty Cycle
#define Scan_Limit_Reg      0x0B    //Scan-Limit Register Format (Table 8)
#define Scan_Limit          0x07    //Display digits 7-0
#define Shutdown_Reg_Format 0x0C    // Shutdown Register Format (Table 3)
#define Shutdown_Format     0x01    // Normal Operation
#define Display_Test_Reg    0x0F    // Display-Test Register Format (Table 10) 
#define Display_Test        0x00    // Mode Normal Operation
/* Define for MPU6050 --------------------------------------------------------------------------*/
#define INT_pin             7
#define Who_Am_I            0x75    //Register 117 – Who Am I
#define Signal_Path_Reset   0x68    //Register 104 – Signal Path Reset
#define SMPRT_DIV           0x19    //Register 25 – Sample Rate Divider
#define CONFIG              0x1A    //Register 26 – Configuration (DLPF_CFG)
#define GYRO_CONFIG         0x1B    //Register 27 – Gyroscope Configuration
#define ACCEL_CONFIG        0x1C    //Register 28 – Accelerometer Configuration
#define INT_ENABLE          0x38    //Register 56 – Interrupt Enable
#define PWR_MGMT_1          0x6B    //Register 107 – Power Management 1
#define INT_STATUS          0x3A    //Register 58 – Interrupt Status
#define TEMP_OUT_H          0x41    //Registers 65 – Temperature Measurement
#define pi 3.14159
#define ACC_X 		  59
#define ACC_Y 		  61
#define ACC_Z		  63
uint16_t temp;
int mpu;
uint8_t right[8] = {0x00,0x08,0x04,0x7E,0x04,0x08,0x00,0x00};
uint8_t up[8] = {0x00,0x08,0x1C,0x2A,0x08,0x08,0x08,0x00};
uint8_t left[8] = {0x00,0x10,0x20,0x7E,0x20,0x10,0x00,0x00};
uint8_t down[8] = {0x00,0x08,0x08,0x8,0x2A,0x1C,0x08,0x00};

void initMpu(void){
	wiringPiI2CWriteReg8(mpu, SMPRT_DIV, 0x0F);
	wiringPiI2CWriteReg8(mpu, CONFIG, 0x02);
	wiringPiI2CWriteReg8(mpu, GYRO_CONFIG, 0x08);
	wiringPiI2CWriteReg8(mpu, ACCEL_CONFIG, 0x10);
	wiringPiI2CWriteReg8(mpu, INT_ENABLE, 0x01);
	wiringPiI2CWriteReg8(mpu, PWR_MGMT_1, 0x01);
}

uint8_t buf[2];
//send data to Max7219
void sendData(uint8_t address, uint8_t data)
{
    buf[0] = address;
    buf[1] = data;
    wiringPiSPIDataRW(spi0, buf, 2);
}
int16_t read_sensor(unsigned char sensor){
	int16_t high, low, data;
	high = wiringPiI2CReadReg8(mpu, sensor); 	 
	low = wiringPiI2CReadReg8(mpu, sensor + 1);			
	data = (high<<8) | low; 																							
	return data;
}
void Init_Max7219(void)
{
    sendData(Decode_Mode_Reg, Digits);
    sendData(Intensity_Reg, Intensity);
    sendData(Scan_Limit_Reg, Scan_Limit);
    sendData(Shutdown_Reg_Format, Shutdown_Format);
    sendData(Display_Test_Reg, Display_Test);
}
void arrow(uint8_t * direction)
{
	delay(300);
    for (int i=0;i<8;i++)
            sendData(i+1,direction[7-i]);
}
void led_matrix(float pitch, float roll)
{
        if(pitch>0 & roll >0)
			arrow(up);
        else if(pitch>0 & roll <0)
			arrow(left);
        else if(pitch<0 & roll >0)
			arrow(right);
        else if(pitch<0 & roll <0)
			arrow(down);
}

int main(void){
	// setup i2c interfface
	mpu = wiringPiI2CSetup(Signal_Path_Reset);
    wiringPiSetup();
	// check connection
	if(wiringPiI2CReadReg8(mpu, Who_Am_I)!= Signal_Path_Reset){
		printf("Connection fail. \n");
		exit(1);
	}
	// setup operational mode for mpu6050
	initMpu();
    // setup SPI interface
    wiringPiSPISetup(spi0, 10000000); //Serial-Clock Input 10MHz max
    // set operational mode for max7219
    Init_Max7219();
    while(1)
    {
        float Ax = (float)read_sensor(ACC_X)/4096.0;
		float Ay = (float)read_sensor(ACC_Y)/4096.0;
		float Az = (float)read_sensor(ACC_Z)/4096.0;
		
		float pitch = atan2(Ax,sqrt(pow(Ay,2)+pow(Az,2)))*180/pi;
		float roll = atan2(Ay,sqrt(pow(Ax,2)+pow(Az,2)))*180/pi;

        led_matrix(pitch,roll);
    }

	return 0;
}