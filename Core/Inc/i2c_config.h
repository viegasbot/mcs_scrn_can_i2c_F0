/*
 * i2c_config.h
 *
 *  Created on: 29 mar 2023
 *      Author: patry
 */

#ifndef INC_I2C_CONFIG_H_
#define INC_I2C_CONFIG_H_

enum I2C_MSG_ID
{
	MAP_I2C_ID = 0x3Cu,
	TC_I2C_ID = 0x3Du,
	DIFF_I2C_ID = 0x3Eu,
	TSAC_I2C_ID = 0x3Fu,
	SPEED_I2C_ID = 0x40u,
	LENG_I2C_ID = 0x41u,
	RENG_I2C_ID = 0x42u,
	LINV_I2C_ID = 0x43u,
	RINV_I2C_ID = 0x44u,
	BAT_I2C_ID = 0x45u,
	HV_I2C_ID = 0x46u,
	LOW_I2C_ID = 0x47u,
	ERR_I2C_ID = 0x48u,
	P2D_I2C_ID = 0x49u,
	TS_I2C_ID = 0x4Au
};


#endif /* INC_I2C_CONFIG_H_ */
