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

// Mailbox commands
#define GET_AG_ID       0x1
#define READ_ACCEL       0x10
#define GET_MAG_ID       0x2
#define GET_ALT_ID       0x4


/************************** Function Prototypes ******************************/
void NAV_InitAG(void);
void NAV_InitMAG(void);
void NAV_InitALT(void);
u16 get_device_ag_id(void);
u16 get_device_mag_id(void);
u16 get_device_alt_id(void);
void read_accel(float *X, float *Y, float *Z);

/************************** Global Variables *****************************/
spi acelgir_spi;
spi mag_spi;
spi alt_spi;

int main (void) {
    u32 cmd;

    acelgir_spi = spi_open(3,2,1,0);
    acelgir_spi = spi_configure(acelgir_spi, 0, 0);
	// mag_spi = spi_open(3,2,1,6);
	// mag_spi = spi_configure(mag_spi, 0, 0);
	// alt_spi = spi_open(3,2,1,7);
    // alt_spi = spi_configure(alt_spi, 0, 0);

    NAV_InitAG();
    // NAV_InitMAG();
    // NAV_InitALT();

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
				
//			case GET_MAG_ID:				
//				MAILBOX_DATA(0) = get_device_mag_id();			
//              MAILBOX_CMD_ADDR = 0x0;
//              break;
				
//			case GET_ALT_ID:				
//				MAILBOX_DATA(0) = get_device_alt_id();			
//              MAILBOX_CMD_ADDR = 0x0;
//              break;

	  	default:
				MAILBOX_CMD_ADDR = 0x0;
				break;
            }
    }
    return 0;
}


void read_accel(float *X, float *Y, float *Z){
	
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

}

u16 get_device_ag_id(void){

    WriteBuffer[0] = NAV_ACL_GYRO_WHO_AM_I | 0x80;
    WriteBuffer[1] = 0x00; 
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	spi_transfer(acelgir_spi, (char*)WriteBuffer, (char*)ReadBuffer, 2);
	
	u16 result = ((ReadBuffer[0] << 8) + (ReadBuffer[1]) );
	
	return ( result );
	
}

u16 get_device_mag_id(void){

	WriteBuffer[0] = NAV_MAG_WHO_AM_I_M | 0xC0;
    WriteBuffer[1] = 0x00; 
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	spi_transfer(mag_spi, (char*)WriteBuffer, (char*)ReadBuffer, 2);
	
	u16 result = ((ReadBuffer[0] << 8) + (ReadBuffer[1]) );
	
	return ( result );
}

u16 get_device_alt_id(void){
	
	WriteBuffer[0] = NAV_ALT_WHO_AM_I | 0xC0;
    WriteBuffer[1] = 0x00; 		
	ReadBuffer[0] = 0x00;
	ReadBuffer[1] = 0x00;
	spi_transfer(alt_spi, (char*)WriteBuffer, (char*)ReadBuffer, 2);
	
	u16 result = ((ReadBuffer[0] << 8) + (ReadBuffer[1]) );
	
	return ( result );
	
}

void NAV_InitAG(void){
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
}

void NAV_InitMAG(void){
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
}

void NAV_InitALT(void){
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
}