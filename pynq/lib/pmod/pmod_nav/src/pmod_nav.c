/******************************************************************************
 *  Copyright (c) 2016, Xilinx, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.  Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *
 *  2.  Redistributions in binary form must reproduce the above copyright 
 *      notice, this list of conditions and the following disclaimer in the 
 *      documentation and/or other materials provided with the distribution.
 *
 *  3.  Neither the name of the copyright holder nor the names of its 
 *      contributors may be used to endorse or promote products derived from 
 *      this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
/******************************************************************************
 *
 *
 * @file pmod_oled.c
 *
 * IOP code (MicroBlaze) for Pmod OLED.
 * Pmod OLED is write only, and has IIC interface.
 * Switch configuration is done within this program, Pmod should
 * be plugged into upper row of connector
 * The PmodOLED is 128x32 pixel monochrome organic LED (OLED) panel powered by
 * SSD1306. Users can program the device through SPI.
 * http://store.digilentinc.com/pmodoled-organic-led-graphic-display/
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- --- ------- -----------------------------------------------
 * 1.00a gs  11/19/15 release
 * 1.00b yrq 05/27/16 reduce program size, use pmod_init()
 * 2.10  yrq 01/12/18 io_switch refactor
 *
 * </pre>
 *
 *****************************************************************************/

#include "xparameters.h"
// #include "xil_io.h"
#include "spi.h"
#include "timer.h"
#include "circular_buffer.h"
#include "sleep.h"
#include "PMODNav.h"

#define SPI_BASEADDR XPAR_SPI_0_BASEADDR // base address of QSPI[0]

u8 WriteBuffer[2];
u8 ReadBuffer[2];
u8 ReadAccel[6];

float XAcc, YAcc, ZAcc;
float XGyro, YGyro, ZGyro;
float XMag, YMag, ZMag;
float Alt;
float Temp;

// Mailbox commands
#define GET_AG_ID   	 0x1
#define READ_ACCEL       0x10
#define READ_GYRO        0x15
#define GET_MAG_ID       0x2
#define READ_MAG	     0x20
#define GET_ALT_ID       0x4
#define READ_ALT	     0x40
#define READ_TEMP	     0x45


/************************** Function Prototypes ******************************/
void NAV_InitAG(void);
void NAV_InitMAG(void);
void NAV_InitALT(void);
u16 get_device_ag_id(void);
u16 get_device_mag_id(void);
u16 get_device_alt_id(void);
void read_accel(float *X, float *Y, float *Z);
void read_gyro(float *X, float *Y, float *Z);
void read_mag(float *X, float *Y, float *Z);
void read_alt(float *X);
void read_temp(float *X);

/************************** Global Variables *****************************/
spi acelgir_spi;
spi mag_spi;
spi alt_spi;

int main (void) {
    u32 cmd;

    NAV_InitAG();
    NAV_InitMAG();
    NAV_InitALT();
	
    while(1){
        while(MAILBOX_CMD_ADDR==0);
        cmd = MAILBOX_CMD_ADDR;
        
        switch(cmd){
            case GET_AG_ID:
				MAILBOX_DATA(0) = get_device_ag_id();
                MAILBOX_CMD_ADDR = 0x0;
                break;
				
			case READ_ACCEL:
				read_accel(&XAcc, &YAcc, &ZAcc);
				MAILBOX_DATA(0) = XAcc;
				MAILBOX_DATA(1) = YAcc;
				MAILBOX_DATA(2) = ZAcc;
                MAILBOX_CMD_ADDR = 0x0;
                break;

			case READ_GYRO:
				read_gyro(&XGyro, &YGyro, &ZGyro);
				MAILBOX_DATA(0) = XGyro;
				MAILBOX_DATA(1) = YGyro;
				MAILBOX_DATA(2) = ZGyro;
                MAILBOX_CMD_ADDR = 0x0;
                break;

			case GET_MAG_ID:
				MAILBOX_DATA(0) = get_device_mag_id();	
				MAILBOX_CMD_ADDR = 0x0;
				break;
				
			case READ_MAG:
				read_mag(&XMag, &YMag, &ZMag);
				MAILBOX_DATA(0) = XMag;
				MAILBOX_DATA(1) = YMag;
				MAILBOX_DATA(2) = ZMag;
                MAILBOX_CMD_ADDR = 0x0;
                break;
				
			case GET_ALT_ID:
				MAILBOX_DATA(0) = get_device_alt_id();			
                MAILBOX_CMD_ADDR = 0x0;
                break;
				
			case READ_ALT:
				read_alt(&Alt);
				MAILBOX_DATA(0) = Alt;
                MAILBOX_CMD_ADDR = 0x0;
                break;
				
			case READ_TEMP:
				read_temp(&Temp);
				MAILBOX_DATA(0) = Temp;
                MAILBOX_CMD_ADDR = 0x0;
                break;
				
	  	default:
				MAILBOX_CMD_ADDR = 0x0;
				break;
            }
    }
    return 0;
}



/*****
	ACCELEROMETER/GIROSCOPE
*****/
void NAV_InitAG(void){
	
	// Configure SPI
	acelgir_spi = spi_open(3,2,1,0);
	acelgir_spi = spi_configure(acelgir_spi, 0, 0);
	
	// Enable all three axes
	WriteBuffer[0] = NAV_ACL_CTRL_REG5_XL | 0x00;
    WriteBuffer[1] = 0x38;
    spi_transfer(acelgir_spi, (char*)WriteBuffer, NULL, 2);
	
	// Set 10Hz odr for accel when used together with gyro
	WriteBuffer[0] = NAV_ACL_CTRL_REG6_XL | 0x00;
    WriteBuffer[1] = 0x20;
    spi_transfer(acelgir_spi, (char*)WriteBuffer, NULL, 2);
		
	// Set 10Hz rate for Gyro
	WriteBuffer[0] = NAV_GYRO_CTRL_REG1_G | 0x00;
    WriteBuffer[1] = 0x20;
    spi_transfer(acelgir_spi, (char*)WriteBuffer, NULL, 2);
	
	// Enable the axes outputs for Gyro
	WriteBuffer[0] = NAV_ACL_GYRO_CTRL_REG4 | 0x00;
    WriteBuffer[1] = 0x38;
	spi_transfer(acelgir_spi, (char*)WriteBuffer, NULL, 2);
	
	// Close SPI
	spi_close(acelgir_spi);
}

u16 get_device_ag_id(void){

	// Configure SPI
	acelgir_spi = spi_open(3,2,1,0);
	acelgir_spi = spi_configure(acelgir_spi, 0, 0);

    WriteBuffer[0] = NAV_ACL_GYRO_WHO_AM_I | 0x80;
    WriteBuffer[1] = 0x00; 
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	spi_transfer(acelgir_spi, (char*)WriteBuffer, (char*)ReadBuffer, 2);
	
	u16 result = ((ReadBuffer[0] << 8) + (ReadBuffer[1]) );
	
	// Close SPI
	spi_close(acelgir_spi);
	
	return ( result );
}


void read_accel(float *X, float *Y, float *Z){
	
	// Configure SPI
	acelgir_spi = spi_open(3,2,1,0);
	acelgir_spi = spi_configure(acelgir_spi, 0, 0);

	u8 AclX_L, AclX_H, AclY_L, AclY_H, AclZ_L, AclZ_H;
	s16 AclX, AclY, AclZ;
	
    WriteBuffer[0] = NAV_ACL_OUT_X_L_XL | 0x80;
    WriteBuffer[1] = 0x00; 
	
	ReadAccel[0] = 0x00;
	ReadAccel[1] = 0x00;
	ReadAccel[2] = 0x00;
	ReadAccel[3] = 0x00;
	ReadAccel[4] = 0x00;
	ReadAccel[5] = 0x00;
	
	spi_transfer(acelgir_spi, (char*)WriteBuffer, (char*)ReadAccel, 6+1);
	
	AclX_L = ReadAccel[0];
	AclX_H = ReadAccel[1];
	AclY_L = ReadAccel[2];
	AclY_H = ReadAccel[3];
	AclZ_L = ReadAccel[4];
	AclZ_H = ReadAccel[5];
	
	// Combines the read values for each axis to obtain the 16-bit values
	AclX = (s16) (((s16) AclX_H << 8) | AclX_L);
	AclY = (s16) ((s16) AclY_H << 8) | AclY_L;
	AclZ = (s16) ((s16) AclZ_H << 8) | AclZ_L;
	
	*X = ((float) AclX) * 0.000061; // NAV_ACL_PAR_XL_2G
	*Y = ((float) AclY) * 0.000061; // NAV_ACL_PAR_XL_2G
	*Z = ((float) AclZ) * 0.000061; // NAV_ACL_PAR_XL_2G
	
	// Close SPI
	spi_close(acelgir_spi);
}

void read_gyro(float *X, float *Y, float *Z){
	
	// Configure SPI
	acelgir_spi = spi_open(3,2,1,0);
	acelgir_spi = spi_configure(acelgir_spi, 0, 0);

	u8 iGX_L, iGX_H, iGY_L, iGY_H, iGZ_L, iGZ_H;
	s16 GX, GY, GZ;
	
    WriteBuffer[0] = NAV_GYRO_OUT_X_L_G | 0x80;
    WriteBuffer[1] = 0x00; 
	
	ReadAccel[0] = 0x00;
	ReadAccel[1] = 0x00;
	ReadAccel[2] = 0x00;
	ReadAccel[3] = 0x00;
	ReadAccel[4] = 0x00;
	ReadAccel[5] = 0x00;
	
	spi_transfer(acelgir_spi, (char*)WriteBuffer, (char*)ReadAccel, 6+1);
	
	iGX_L = ReadAccel[0];
	iGX_H = ReadAccel[1];
	iGY_L = ReadAccel[2];
	iGY_H = ReadAccel[3];
	iGZ_L = ReadAccel[4];
	iGZ_H = ReadAccel[5];
	
	// Combines the read values for each axis to obtain the 16-bit values
	GX = (s16) (((s16) iGX_H << 8) | iGX_L);
	GY = (s16) ((s16) iGY_H << 8) | iGY_L;
	GZ = (s16) ((s16) iGZ_H << 8) | iGZ_L;
	
	*X = ((float) GX) * 0.00875; // NAV_GYRO_PAR_G_245DPS
	*Y = ((float) GY) * 0.00875; // NAV_GYRO_PAR_G_245DPS
	*Z = ((float) GZ) * 0.00875; // NAV_GYRO_PAR_G_245DPS
	
	// Close SPI
	spi_close(acelgir_spi);
}


/*****
	MAGNETOMETER
*****/

void NAV_InitMAG(void){
	
	// Configure SPI
	mag_spi = spi_open(3,2,1,6);
	mag_spi = spi_configure(mag_spi, 0, 0);
	
	// Set medium performance mode for x and y and 10Hz ODR for MAG,
	WriteBuffer[0] = NAV_MAG_CTRL_REG1_M | 0x00;
    WriteBuffer[1] = 0x30;
    spi_transfer(mag_spi, (char*)WriteBuffer, NULL, 2);
	
	// Set scale to +-4Gauss
	WriteBuffer[0] = NAV_MAG_CTRL_REG2_M | 0x00;
    WriteBuffer[1] = 0x00;
    spi_transfer(mag_spi, (char*)WriteBuffer, NULL, 2);
	
	// Disable I2C and enable SPI read and write operations,
	// Set the operating mode to continuous conversion
	WriteBuffer[0] = NAV_MAG_CTRL_REG3_M | 0x00;
    WriteBuffer[1] = 0x00;
    spi_transfer(mag_spi, (char*)WriteBuffer, NULL, 2);
	
	// Set medium performance mode for z axis
	WriteBuffer[0] = NAV_MAG_CTRL_REG4_M | 0x00;
    WriteBuffer[1] = 0x04;
    spi_transfer(mag_spi, (char*)WriteBuffer, NULL, 2);
	
	// Continuous update of output registers
	WriteBuffer[0] = NAV_MAG_CTRL_REG5_M | 0x00;
    WriteBuffer[1] = 0x00;
    spi_transfer(mag_spi, (char*)WriteBuffer, NULL, 2);
	
	// close SPI
	spi_close(mag_spi);
}


u16 get_device_mag_id(void){
	
	// Configure SPI
	mag_spi = spi_open(3,2,1,6);
	mag_spi = spi_configure(mag_spi, 0, 0);

	WriteBuffer[0] = NAV_MAG_WHO_AM_I_M | 0xC0;
    WriteBuffer[1] = 0x00; 
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	spi_transfer(mag_spi, (char*)WriteBuffer, (char*)ReadBuffer, 2);
	
	u16 result = ((ReadBuffer[0] << 8) + (ReadBuffer[1]) );
	
	// close SPI
	spi_close(mag_spi);
	
	return ( result );
}

void read_mag(float *X, float *Y, float *Z){
	
	// Configure SPI
	mag_spi = spi_open(3,2,1,6);
	mag_spi = spi_configure(mag_spi, 0, 0);

	u8 iMagX_L, iMagX_H, iMagY_L, iMagY_H, iMagZ_L, iMagZ_H;
	s16 MagX, MagY, MagZ;
	
	// ToDo: Check status
	
    WriteBuffer[0] = NAV_MAG_OUT_X_L_M | 0xC0;
    WriteBuffer[1] = 0x00; 
	
	ReadAccel[0] = 0x00;
	ReadAccel[1] = 0x00;
	ReadAccel[2] = 0x00;
	ReadAccel[3] = 0x00;
	ReadAccel[4] = 0x00;
	ReadAccel[5] = 0x00;
	
	spi_transfer(acelgir_spi, (char*)WriteBuffer, (char*)ReadAccel, 6+1);
	
	iMagX_L = ReadAccel[0];
	iMagX_H = ReadAccel[1];
	iMagY_L = ReadAccel[2];
	iMagY_H = ReadAccel[3];
	iMagZ_L = ReadAccel[4];
	iMagZ_H = ReadAccel[5];
	
	// Combines the read values for each axis to obtain the 16-bit values
	MagX = (s16) (((s16) iMagX_H << 8) | iMagX_L);
	MagY = (s16) ((s16) iMagY_H << 8) | iMagY_L;
	MagZ = (s16) ((s16) iMagZ_H << 8) | iMagZ_L;
	
	*X = ((float) MagX) * 0.00014; // NAV_MAG_PAR_MAG_4GAUSS
	*Y = ((float) MagY) * 0.00014; // NAV_MAG_PAR_MAG_4GAUSS
	*Z = ((float) MagZ) * 0.00014; // NAV_MAG_PAR_MAG_4GAUSS
	
	// Close SPI
	spi_close(mag_spi);
}

/*****
	ALTIMETER
*****/
void NAV_InitALT(void){
	
	// Configure SPI
	alt_spi = spi_open(3,2,1,7);
	alt_spi = spi_configure(alt_spi, 0, 0);
	
	// Clean start
	WriteBuffer[0] = NAV_ALT_CTRL_REG1 | 0x00;
    WriteBuffer[1] = 0x00;
    spi_transfer(alt_spi, (char*)WriteBuffer, NULL, 2);
	usleep(5000);
	
	// Set active the device and ODR to 7Hz
	WriteBuffer[0] = NAV_ALT_CTRL_REG1 | 0x00;
    WriteBuffer[1] = 0xA4;
    spi_transfer(alt_spi, (char*)WriteBuffer, NULL, 2);
	usleep(5000);
	
	// Increment address during multiple byte access disabled
	WriteBuffer[0] = NAV_ALT_CTRL_REG2 | 0x00;
    WriteBuffer[1] = 0x00;
    spi_transfer(alt_spi, (char*)WriteBuffer, NULL, 2);
	usleep(5000);
	
	// No modification to interrupt sources
	WriteBuffer[0] = NAV_ALT_CTRL_REG4 | 0x00;
    WriteBuffer[1] = 0x00;
    spi_transfer(alt_spi, (char*)WriteBuffer, NULL, 2);
	usleep(5000);
	
	// Close SPI
	spi_close(alt_spi);	
}

u16 get_device_alt_id(void){
	// Configure SPI
	alt_spi = spi_open(3,2,1,7);
	alt_spi = spi_configure(alt_spi, 0, 0);
	
	WriteBuffer[0] = NAV_ALT_WHO_AM_I | 0xC0;
    WriteBuffer[1] = 0x00; 		
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	spi_transfer(alt_spi, (char*)WriteBuffer, (char*)ReadBuffer, 2);
	
	u16 result = ((ReadBuffer[0] << 8) + (ReadBuffer[1]) );
	
	// Close SPI
	spi_close(alt_spi);	
	
	return ( result );
	
}

void read_alt(float *X){
	
	// Configure SPI
	alt_spi = spi_open(3,2,1,7);
	alt_spi = spi_configure(alt_spi, 0, 0);

	u8 iPress_XL, iPress_L, iPress_H;
	int dataPress = 0;
	
	// ToDo: Check status
	
    WriteBuffer[0] = NAV_ALT_PRESS_OUT_XL | 0xC0;
    WriteBuffer[1] = 0x00; 
	
	ReadAccel[0] = 0x00;
	ReadAccel[1] = 0x00;
	ReadAccel[2] = 0x00;
	
	spi_transfer(alt_spi, (char*)WriteBuffer, (char*)ReadAccel, 3+1);
	
	iPress_XL = ReadAccel[0];
	iPress_L  = ReadAccel[1];
	iPress_H  = ReadAccel[2];
	
	// Combines the read values for each axis to obtain the 16-bit values
	dataPress = (iPress_H << 16) | iPress_L << 8 | iPress_XL;
	
	// Check if there is a negative value
	if (dataPress & 0x00800000) {
		dataPress |= 0xFF000000;
	}
	
	*X = ((float)dataPress / 4096.0);
	
	// Close SPI
	spi_close(alt_spi);
}

void read_temp(float *X){
	
	// Configure SPI
	alt_spi = spi_open(3,2,1,7);
	alt_spi = spi_configure(alt_spi, 0, 0);

	u8 tempL, tempH;
	
	// ToDo: Check status
    WriteBuffer[0] = NAV_ALT_TEMP_OUT_L | 0xC0;
    WriteBuffer[1] = 0x00; 
	
	ReadAccel[0] = 0x00;
	ReadAccel[1] = 0x00;
	
	spi_transfer(alt_spi, (char*)WriteBuffer, (char*)ReadAccel, 2+1);
	
	tempL = ReadAccel[0];
	tempH = ReadAccel[1];
		
	// Combines the read values for each axis to obtain the 16-bit values
	s16 temp = (s16) (tempH << 8 | tempL);
	
	*X = 42.5 + (temp * 0.002083);
	
	// Close SPI
	spi_close(alt_spi);
}