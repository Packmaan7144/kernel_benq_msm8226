/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#define OV8865_SENSOR_NAME "ov8865"
DEFINE_MSM_MUTEX(ov8865_mut);

static struct msm_sensor_ctrl_t ov8865_s_ctrl;
static struct otp_struct_q st_ov8865_otp;

#define  LSC_REG_NUM 62
static uint16_t ov8865_lsc_data[LSC_REG_NUM] ={0x00};

#define OTP_EMPTY 0
#define OTP_INVALID 1
#define OTP_VALID 2

static int initOtp = false; 

static struct msm_sensor_power_setting ov8865_power_setting[] = {
	{ 
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{ 
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
#if 0
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
#if 0
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
#endif
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov8865_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8865_i2c_id[] = {
	{OV8865_SENSOR_NAME, (kernel_ulong_t)&ov8865_s_ctrl},
	{ }
};

static int32_t msm_ov8865_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8865_s_ctrl);
}

static struct i2c_driver ov8865_i2c_driver = {
	.id_table = ov8865_i2c_id,
	.probe  = msm_ov8865_i2c_probe,
	.driver = {
		.name = OV8865_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8865_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8865_dt_match[] = {
	{.compatible = "ovti,ov8865", .data = &ov8865_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8865_dt_match);

static struct platform_driver ov8865_platform_driver = {
	.driver = {
		.name = "ovti,ov8865",
		.owner = THIS_MODULE,
		.of_match_table = ov8865_dt_match,
	},
};

static int32_t ov8865_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov8865_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov8865_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8865_platform_driver,
		ov8865_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov8865_i2c_driver);
}

static void __exit ov8865_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8865_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8865_s_ctrl);
		platform_driver_unregister(&ov8865_platform_driver);
	} else
		i2c_del_driver(&ov8865_i2c_driver);
	return;
}

int ov8865_read_otp_module_id(struct msm_sensor_ctrl_t* s_ctrl)
{
	int i = 0;
	int rc = 0;
	uint16_t temp;

	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d84,
		              0xc0, MSM_CAMERA_I2C_BYTE_DATA);

	
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d88,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d89,
		              0x00, MSM_CAMERA_I2C_BYTE_DATA);
	
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8a,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8b,
		              0x0f, MSM_CAMERA_I2C_BYTE_DATA);

       
	rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d81,
		              0x01, MSM_CAMERA_I2C_BYTE_DATA);
       msleep(5);

	for (i = 0; i < 16; i++)
	{
		rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					0x7000+i, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

		st_ov8865_otp.module_info.module_id[i] = (uint8_t) temp;
	}

	return rc ;

}

int ov8865_check_otp_info_index(struct msm_sensor_ctrl_t* s_ctrl, int index)
{
	uint16_t flag = 0xff;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d84,
		              0xc0, MSM_CAMERA_I2C_BYTE_DATA);

	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d88,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d89,
		              0x10, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8a,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8b,
		              0x10, MSM_CAMERA_I2C_BYTE_DATA);

       
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d81,
		              0x01, MSM_CAMERA_I2C_BYTE_DATA);
       msleep(5);

       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					0x7010, &flag,
					MSM_CAMERA_I2C_BYTE_DATA);

	if( index == 0)
		flag = (flag >> 6) & 0x03;
	else if ( index == 1)
		flag = (flag >> 4) & 0x03;
	else if ( index == 2)
		flag = (flag >> 2) & 0x03;

       
       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x7010,
		              0x00, MSM_CAMERA_I2C_BYTE_DATA);

	if( flag == 0x00 ) 
		return OTP_EMPTY;
	else if ( flag == 0x02 || flag ==0x03 ) 
		return OTP_INVALID;
	else 
		return OTP_VALID;
}


int ov8865_check_otp_awb_index(struct msm_sensor_ctrl_t* s_ctrl, int index)
{
	uint16_t flag = 0xff;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d84,
		              0xc0, MSM_CAMERA_I2C_BYTE_DATA);

	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d88,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d89,
		              0x20, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8a,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8b,
		              0x20, MSM_CAMERA_I2C_BYTE_DATA);

       
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d81,
		              0x01, MSM_CAMERA_I2C_BYTE_DATA);
       msleep(5);

       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					0x7020, &flag,
					MSM_CAMERA_I2C_BYTE_DATA);

	if( index == 0)
		flag = (flag >> 6) & 0x03;
	else if ( index == 1)
		flag = (flag >> 4) & 0x03;
	else if ( index == 2)
		flag = (flag >> 2) & 0x03;

       
       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x7020,
		              0x00, MSM_CAMERA_I2C_BYTE_DATA);

	if( flag == 0x00 ) 
		return OTP_EMPTY;
	else if ( flag == 0x02  || flag ==0x03 ) 
		return OTP_INVALID;
	else 
		return OTP_VALID;

}


int ov8865_check_otp_lsc_index(struct msm_sensor_ctrl_t* s_ctrl, int index)
{
	uint16_t flag = 0xff;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d84,
		              0xc0, MSM_CAMERA_I2C_BYTE_DATA);

	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d88,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d89,
		              0x3a, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8a,
		              0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8b,
		              0x3a, MSM_CAMERA_I2C_BYTE_DATA);

       
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d81,
		              0x01, MSM_CAMERA_I2C_BYTE_DATA);
       msleep(5);

       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					0x703a, &flag,
					MSM_CAMERA_I2C_BYTE_DATA);

	if( index == 0)
		flag = (flag >> 6) & 0x03;
	else if ( index == 1)
		flag = (flag >> 4) & 0x03;
	else if ( index == 2)
		flag = (flag >> 2) & 0x03;

       
       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x703a,
		              0x00, MSM_CAMERA_I2C_BYTE_DATA);

	if( flag == 0x00 ) 
		return OTP_EMPTY;
	else if ( flag == 0x02 || flag ==0x03 ) 
		return OTP_INVALID;
	else 
		return OTP_VALID;
}
int ov8865_read_otp_sensor_info_data ( struct msm_sensor_ctrl_t* s_ctrl, int index)
{
	int i = 0, rc = 0;
	int star_addr, end_addr;
	uint16_t temp;

	if(index == 0){
		star_addr = 0x7011;
		end_addr = 0x7015;
	}
	else if (index == 1){
		star_addr = 0x7016;
		end_addr = 0x701A;
	}
	else if (index ==2 ){
		star_addr = 0x701B;
		end_addr = 0x701F;
	}
	else{

		pr_err("%s, wrong index :%d\n", __func__, index);
		return -1;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d84,
		              0xc0, MSM_CAMERA_I2C_BYTE_DATA);

	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d88,
		              (star_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d89,
		              star_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8a,
		              (end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8b,
		              end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);

       
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d81,
		              0x01, MSM_CAMERA_I2C_BYTE_DATA);
       msleep(5);

       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					star_addr, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

       st_ov8865_otp.module_integrator_id = temp;

       s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					star_addr+1, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

	st_ov8865_otp.lens_id= temp;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					star_addr+2, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

	st_ov8865_otp.year= temp;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					star_addr+3, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

	st_ov8865_otp.month= temp;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					star_addr+4, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

	st_ov8865_otp.date= temp;

	for( i = star_addr; i <= end_addr; i++){
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              i, 00, MSM_CAMERA_I2C_BYTE_DATA);
	}

       pr_err("%s, module_integrator_id:0x%x, lens_id:0x%x, year:0x%x, month:0x%x, date: 0x%x\n"
              , __func__, st_ov8865_otp.module_integrator_id, st_ov8865_otp.lens_id, st_ov8865_otp.year,
              st_ov8865_otp.month, st_ov8865_otp.date);

       return rc;

}

int ov8865_read_otp_info_awb_data( struct msm_sensor_ctrl_t* s_ctrl, int index)
{
	int j = 0, max_reg = 5;
	int start_addr, end_addr;
	uint16_t temp[max_reg];

       if ( index >=3 && index < 0){
		pr_err("%s, wrong index :%d\n", __func__, index);
		return -1;
       }

	
	start_addr = 0x7021 + index*5;
	end_addr = 0x7025 + index *5;

	

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3d84,
			0xc0, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3d88,
			(start_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3d89,
			start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3d8a,
			(end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3d8b,
			end_addr & 0xff , MSM_CAMERA_I2C_BYTE_DATA);

	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x3d81,
			0x01, MSM_CAMERA_I2C_BYTE_DATA);

	

	msleep(15);

	
	for( j = 0; j < max_reg; j++){
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			start_addr+j, &temp[0+j],
			MSM_CAMERA_I2C_BYTE_DATA);
			
	}

	
	st_ov8865_otp.rg_ratio_d50 = (temp[0] <<2) + ((temp[4] >>6) & 0x3);
	st_ov8865_otp.bg_ratio_d50 = (temp[1] <<2) + ((temp[4] >>4) & 0x3);
	st_ov8865_otp.light_rg_ratio_d50 = (temp[2] << 2) +(( temp[4] >> 2) & 0x3);
	st_ov8865_otp.light_bg_ratio_d50 = (temp[3] << 2) +( temp[4] & 0x3);
	pr_err("%s, rg_ratio_d50:%d, bg_ratio_d50:%d, light_rg_ratio_d50:%d, light_bg_ratio_d50:%d\n",
		__func__, st_ov8865_otp.rg_ratio_d50, st_ov8865_otp.bg_ratio_d50,
		st_ov8865_otp.light_rg_ratio_d50, st_ov8865_otp.light_bg_ratio_d50);

	
	for( j = 0; j < max_reg; j++)
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			start_addr+j,
			0x00,
			MSM_CAMERA_I2C_BYTE_DATA);

     return 0;
}

int ov8865_read_otp_lsc( struct msm_sensor_ctrl_t *s_ctrl , int index)
{
	int i = 0;
	int start_addr = 0, end_addr = 0;
	int rc = 0;

	if( index == 0) {
		start_addr = 0x703B;
		end_addr = 0x7078;
	}
	else if ( index == 1) {
		start_addr = 0x7079;
		end_addr = 0x70B6;
	}
	else if ( index == 2) {
		start_addr = 0x70B7;
		end_addr = 0x70F4;
	}
	else {

		pr_err("%s, wrong index:%d\n", __func__, index);
		return -1;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d84,
		              0xc0, MSM_CAMERA_I2C_BYTE_DATA);

	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d88,
		              (start_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d89,
		              start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8a,
		              (end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d8b,
		              end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);

       
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x3d81,
		              0x01, MSM_CAMERA_I2C_BYTE_DATA);
       msleep(10);

       for ( i = 0; i < LSC_REG_NUM; i++) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				start_addr+i, &ov8865_lsc_data[i],
				MSM_CAMERA_I2C_BYTE_DATA);

			pr_err("%s, %d, lsc:0x%x\n", __func__, i, ov8865_lsc_data[i] );
       }

       for ( i = start_addr; i <= end_addr; i++)
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              i,
		              0x00, MSM_CAMERA_I2C_BYTE_DATA);

       return rc;
}

int ov8865_read_sensor_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i = 0;
	int  otp_module_info_index= -1,otp_awb_index = -1, otp_lsc_index = -1;
       int rc = 0, flag ;

       if(initOtp == true)
	       return 0;

       initOtp = true;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		              s_ctrl->sensor_i2c_client,
		              0x0100,
		               0x01, MSM_CAMERA_I2C_BYTE_DATA);

	rc = ov8865_read_otp_module_id(s_ctrl);

	if( rc < 0)
		pr_err("%s, failed to read OV private ID\n", __func__);

       
	for( i =0; i < 3; i++) {
		flag = ov8865_check_otp_info_index(s_ctrl, i);
		if( flag == OTP_VALID){
			otp_module_info_index = i;
			break;
		}
	}
	if( otp_module_info_index == -1){
		pr_err("%s, there is no valid awb group", __func__);
	}
	else  {
		pr_err("%s, the otp_module_info_index is %d\n", __func__, otp_module_info_index);
	}

       
	for( i =0; i < 3; i++) {
		flag = ov8865_check_otp_awb_index(s_ctrl, i);
		if( flag == OTP_VALID){
			otp_awb_index = i;
			break;
		}
	}

	if( otp_awb_index == -1)
		pr_err("%s, there is no valid awb group", __func__);
	else
		pr_err("%s, the otp_awb_index is %d\n", __func__, otp_awb_index);


       
	for( i =0; i < 3; i++) {
		flag = ov8865_check_otp_lsc_index(s_ctrl, i);
		if( flag == OTP_VALID){
			otp_lsc_index = i;
			break;
		}
	}
	if( otp_lsc_index == -1)
		pr_err("%s, there is no valid lsc group", __func__);
	else
		pr_err("%s, the otp_lsc_index is %d\n", __func__, otp_lsc_index);

	rc |= ov8865_read_otp_sensor_info_data(s_ctrl, otp_module_info_index);

	rc |= ov8865_read_otp_info_awb_data(s_ctrl, otp_awb_index);

	rc |= ov8865_read_otp_lsc(s_ctrl,  otp_lsc_index);

	if( rc == 0)
		st_ov8865_otp.do_calibration = 1;

	return rc;
}

int32_t ov8865_get_calib_params(struct msm_sensor_ctrl_t *s_ctrl, struct otp_params* otp_params)
{
	int rc = 0;

	pr_err("%s, enter\n", __func__);

	otp_params->size = sizeof(struct otp_struct_q);

	memcpy(otp_params->otp_data, &st_ov8865_otp, sizeof(struct otp_struct_q));

	pr_err("%s, rc:%d, size = %d\n", __func__, rc, otp_params->size);
	return rc;

}

int ov8865_write_lsc_calibration(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t rc = 0, i=0;
	uint16_t temp;


	for(i =0; i<LSC_REG_NUM; i++)
	{
			rc |= s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
					0x5800+i,
					ov8865_lsc_data[i],
					MSM_CAMERA_I2C_BYTE_DATA);
	}

	 if(rc == 0 && (st_ov8865_otp.do_calibration == 1))
	 {
	      
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,
					0x5000, &temp,
					MSM_CAMERA_I2C_BYTE_DATA);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
			0x5000,
			(temp | 0x80), 
			MSM_CAMERA_I2C_BYTE_DATA);
	 }

	 return rc;
}

static struct msm_sensor_fn_t ov8865_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	
	.sensor_read_otp = ov8865_read_sensor_otp,
	.sensor_get_calib_params = ov8865_get_calib_params,
	.sensor_write_lsc_calibration = ov8865_write_lsc_calibration,
	
};

static struct msm_sensor_ctrl_t ov8865_s_ctrl = {
	.sensor_i2c_client = &ov8865_sensor_i2c_client,
	.power_setting_array.power_setting = ov8865_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov8865_power_setting),
	.msm_sensor_mutex = &ov8865_mut,
	.sensor_v4l2_subdev_info = ov8865_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8865_subdev_info),
	.func_tbl = &ov8865_sensor_func_tbl,
	.otp_info = &st_ov8865_otp,
};

module_init(ov8865_init_module);
module_exit(ov8865_exit_module);
MODULE_DESCRIPTION("ov8865");
MODULE_LICENSE("GPL v2");
