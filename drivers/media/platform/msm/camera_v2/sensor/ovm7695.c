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
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define OVM7695_SENSOR_NAME "ovm7695"
#define PLATFORM_DRIVER_NAME "msm_camera_ovm7695"
#define ovm7695_obj ovm7695_##obj


#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif



#define MSG2(format, arg...)     {}
#define MSG3(format, arg...)  printk(KERN_ERR "[CAM]" format "\n", ## arg)


DEFINE_MSM_MUTEX(ovm7695_mut);
static struct msm_sensor_ctrl_t ovm7695_s_ctrl;

static struct msm_sensor_power_setting ovm7695_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{ 
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
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

static struct msm_camera_i2c_reg_conf ovm7695_recommend_settings[] = {
#if 1 
  {0x0103, 0x01},

  {0x3620, 0x2f},
  {0x3623, 0x12},
  {0x3718, 0x88},
  {0x3703, 0x80},
  {0x3712, 0x40},
  {0x3706, 0x40},
  {0x3631, 0x44},
  {0x3632, 0x05},
  {0x3013, 0xd0},
  {0x3705, 0x1d},
  {0x3713, 0x0e},
  {0x3012, 0x0a},
  {0x3717, 0x18}, 
  {0x3621, 0x47}, 
  {0x0309, 0x24},
  {0x3820, 0x90},
  {0x4803, 0x08},
  {0x0101, 0x01},
  {0x5100, 0x01},

  {0x4500, 0x24}, 


  
  {0x5301, 0x05},
  {0x5302, 0x0c},
  {0x5303, 0x1c},
  {0x5304, 0x2a},
  {0x5305, 0x39},
  {0x5306, 0x45},
  {0x5307, 0x52},
  {0x5308, 0x5d},
  {0x5309, 0x68},
  {0x530a, 0x7f},
  {0x530b, 0x91},
  {0x530c, 0xa5},
  {0x530d, 0xc6},
  {0x530e, 0xde},
  {0x530f, 0xef},
  {0x5310, 0x16},
  {0x520a, 0xf4},
  {0x520b, 0xf4},
  {0x520c, 0xf4},
  {0x5504, 0x08},
  {0x5505, 0x48},
  {0x5506, 0x07},
  {0x5507, 0x0b},
  {0x3a18, 0x01},
  {0x3a19, 0x00},
  {0x3503, 0x03},
  {0x3500, 0x00},
  {0x3501, 0x21},
  {0x3502, 0x00},
  {0x350a, 0x00},
  {0x350b, 0x00},
  {0x4008, 0x02},
  {0x4009, 0x09},
  {0x3002, 0x09},
  {0x3024, 0x00},
  {0x3503, 0x00},



  {0x0101, 0x01}, 

  {0x5002, 0x48}, 
  {0x5910, 0x00}, 
  {0x3a0f, 0x58}, 
  {0x3a10, 0x50}, 
  {0x3a1b, 0x5a}, 
  {0x3a1e, 0x4e}, 
  {0x3a11, 0xa0}, 
  {0x3a1f, 0x28}, 
  {0x3a18, 0x00},
  {0x3a19, 0xf8}, 
  {0x3503, 0x00}, 
  {0x3a0d, 0x04}, 

  {0x5000, 0xff}, 
  {0x5001, 0x3f}, 


  

  

  

  
  {0x520a, 0xf4}, 
  {0x520b, 0xf4}, 
  {0x520c, 0xf4}, 
  {0x5004, 0x41}, 
  {0x5006, 0x41},  

  
  {0x5301, 0x05}, 
  {0x5302, 0x0c}, 
  {0x5303, 0x1c}, 
  {0x5304, 0x2a}, 
  {0x5305, 0x39}, 
  {0x5306, 0x45}, 
  {0x5307, 0x53}, 
  {0x5308, 0x5d}, 
  {0x5309, 0x68}, 

  {0x530a, 0x7f},  
  {0x530b, 0x91},   
  {0x530c, 0xa5},   
  {0x530d, 0xc6},   
  {0x530e, 0xde},  
  {0x530f, 0xef},   
  {0x5310, 0x16}, 

  
  {0x5003, 0x80}, 
  {0x5500, 0x08}, 
  {0x5501, 0x48}, 
  {0x5502, 0x38}, 
  {0x5503, 0x14}, 
  {0x5504, 0x08}, 
  {0x5505, 0x48}, 
  {0x5506, 0x02}, 
  {0x5507, 0x16}, 
  {0x5508, 0x2d}, 
  {0x5509, 0x08}, 
  {0x550a, 0x48}, 
  {0x550b, 0x06}, 
  {0x550c, 0x04}, 
  {0x550d, 0x01}, 

  
  {0x5800, 0x02}, 
  {0x5803, 0x2e}, 
  {0x5804, 0x20}, 

  
  {0x5600, 0x00}, 
  {0x5601, 0x2e},  
  {0x5602, 0x60}, 
  {0x5603, 0x06}, 
  {0x5604, 0x1c}, 
  {0x5605, 0x65}, 
  {0x5606, 0x81}, 
  {0x5607, 0x9f}, 
  {0x5608, 0x8a}, 
  {0x5609, 0x15}, 
  {0x560a, 0x01}, 
  {0x560b, 0x9c}, 

  {0x3811, 0x07},
  {0x3813, 0x06},


  {0x3630, 0x79},



  
  {0x5000, 0xff}, 
  
  
  {0x5100, 0x01},
  {0x5101, 0x48},
  {0x5102, 0x00},
  {0x5103, 0xf8},
  {0x5104, 0x02},
  {0x5105, 0x00},
  {0x5106, 0x00},
  {0x5107, 0x00},
 
  {0x5108, 0x01},
  {0x5109, 0x48},
  {0x510A, 0x00},
  {0x510B, 0xf8},
  {0x510C, 0x02},
  {0x510D, 0x00},
  {0x510E, 0x01},
  {0x510F, 0x00},
  
  {0x5110, 0x01},
  {0x5111, 0x48},
  {0x5112, 0x00},
  {0x5113, 0xf8},
  {0x5114, 0x02},
  {0x5115, 0x00},
  {0x5116, 0x00},
  {0x5117, 0x00},


  
  {0x3a0f, 0x48}, 
  {0x3a10, 0x40}, 
  {0x3a11, 0x90}, 
  {0x3a1b, 0x4A}, 
  {0x3a1e, 0x3E}, 
  {0x3a1f, 0x18}, 

  
  

  {0x5604, 0x1c}, 
  {0x5605, 0x65}, 
  {0x5606, 0x81}, 
  {0x5607, 0x9f}, 
  {0x5608, 0x8a}, 
  {0x5609, 0x15}, 
  {0x5602, 0x60}, 

  
  {0x5803, 0x2c}, 
  {0x5804, 0x23}, 
  {0x5601, 0x30}, 
  {0x5602, 0x60}, 
  {0x5104, 0x02}, 
  {0x510C, 0x00}, 
  {0x5114, 0x00}, 
  {0x5105, 0x01}, 
  {0x510d, 0x01}, 
  {0x5115, 0x01}, 
  {0x5803, 0x2c}, 
  {0x5804, 0x24}, 



  
  {0x3a00, 0x7c}, 
  {0x382a, 0x08}, 
  {0x3a05, 0x30}, 
  {0x3a17, 0x03}, 
                  
                  
                  
                  
  {0x3a02, 0x07}, 
  {0x3a03, 0xda}, 
  {0x3a14, 0x07}, 
  {0x3a15, 0x80}, 
  {0x3a0d, 0x04}, 
  {0x3a0e, 0x03}, 
  {0x3a21, 0x72}, 



  {0x5104, 0x04}, 


  {0x5310, 0x28}, 
  {0x5301, 0x05}, 
  {0x5302, 0x09}, 
  {0x5303, 0x15}, 
  {0x5304, 0x20}, 
  {0x5305, 0x31}, 
  {0x5306, 0x3f}, 
  {0x5307, 0x50}, 
  {0x5308, 0x5b}, 
  {0x5309, 0x65}, 
  {0x530a, 0x77}, 
  {0x530b, 0x83}, 
  {0x530c, 0x91}, 
  {0x530d, 0xad}, 
  {0x530e, 0xcb}, 
  {0x530f, 0xe2}, 


  {0x5804, 0x24}, 

 {0x3a19, 0xe0}, 

  {0x550d, 0x01}, 
  {0x5506, 0x02}, 
  {0x5507, 0x10}, 


  {0x5502, 0x2c}, 
  {0x5503, 0x04}, 
  {0x5501, 0x1a}, 



  {0x5908, 0x22},	
  {0x5909, 0x22},
  {0x590a, 0xe2},
  {0x590b, 0x2e},
  {0x590c, 0xe2},
  {0x590d, 0x2e},
  {0x590e, 0x22}, 
  {0x590f, 0x22}, 



  {0x4003, 0x08}, 


  {0x5804, 0x14}, 


  {0x550d, 0x01}, 

  {0x5506, 0x03}, 
  {0x5507, 0x15}, 



  {0x5502, 0x3c}, 
  {0x5503, 0x0c}, 

  {0x5501, 0x1a}, 






  {0x5502, 0x10}, 



  {0x5803, 0x38}, 
  {0x5804, 0x38}, 



  {0x0100, 0x01},
#else 

  {0x0103, 0x2f},
  {0x3620, 0x12},
  {0x3623, 0x88},
  {0x3718, 0x80},
  {0x3703, 0x40},
  {0x3712, 0x40},
  {0x3706, 0x44},
  {0x3631, 0x05},
  {0x3632, 0xd0},
  {0x3013, 0x1d},
  {0x3705, 0x0e},
  {0x3713, 0x0a},
  {0x3012, 0x18},
  {0x3717, 0x47}, 
  {0x3621, 0x24}, 
  {0x0309, 0x90},
  {0x3820, 0x08},
  {0x4803, 0x01},
  {0x0101, 0x01},
  {0x5100, 0x01},

  {0x4500, 0x24}, 



  
  {0x5301, 0x05},
  {0x5302, 0x0c},
  {0x5303, 0x1c},
  {0x5304, 0x2a},
  {0x5305, 0x39},
  {0x5306, 0x45},
  {0x5307, 0x52},
  {0x5308, 0x5d},
  {0x5309, 0x68},
  {0x530a, 0x7f},
  {0x530b, 0x91},
  {0x530c, 0xa5},
  {0x530d, 0xc6},
  {0x530e, 0xde},
  {0x530f, 0xef},
  {0x5310, 0x16},
  {0x520a, 0xf4},
  {0x520b, 0xf4},
  {0x520c, 0xf4},
  {0x5504, 0x08},
  {0x5505, 0x48},
  {0x5506, 0x07},
  {0x5507, 0x0b},
  {0x3a18, 0x01},
  {0x3a19, 0x00},
  {0x3503, 0x03},
  {0x3500, 0x00},
  {0x3501, 0x21},
  {0x3502, 0x00},
  {0x350a, 0x00},
  {0x350b, 0x00},
  {0x4008, 0x02},
  {0x4009, 0x09},
  {0x3002, 0x09},
#endif
};

static struct v4l2_subdev_info ovm7695_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static struct msm_camera_i2c_reg_conf ovm7695_config_start_settings[] = {
	{0x0100, 0x01}, 
};

static struct msm_camera_i2c_reg_conf ovm7695_config_stop_settings[] = {
	{0x0100, 0x00}, 
};

static struct msm_camera_i2c_reg_conf ovm7695_config_greenish_settings[] = {
	{0x5200, 0x20}, 
	{0x5204, 0x05}, 
	{0x5208, 0x06}, 
};

static struct msm_camera_i2c_reg_conf ovm7695_config_MWB_settings[] = {
	{0x5200, 0x00}, 
};


static int ovm7695_white_balance_setting = -99;
static int ovm7695_exposure_setting = -99;
static int ovm7695_antibanding_setting = -99;
static int ovm7695_saturation_setting = -99;
static int ovm7695_contrast_setting = -99;
static int ovm7695_sharpness_setting = -99;
static int ovm7695_gain_setting = -99;
static int ovm7695_just_power_up = 0;




static struct msm_camera_i2c_reg_conf ovm7695_auto_wb[] =
{
  {0x5200, 0x00},  
  {0x520a, 0x74},
  {0x520b, 0x64},
  {0x520c, 0xd4},
};
static struct msm_camera_i2c_reg_conf ovm7695_incandscent[] =
{
  {0x5200, 0x20}, 
  {0x5204, 0x04}, 
  {0x5205, 0x00},
  {0x5206, 0x04}, 
  {0x5207, 0xDC},
  {0x5208, 0x0B}, 
  {0x5209, 0xB4},
};
static struct msm_camera_i2c_reg_conf ovm7695_fluorescent[] =  
{
  {0x5200, 0x20}, 
  {0x5204, 0x05},
  {0x5205, 0xA0},
  {0x5206, 0x04},
  {0x5207, 0x00},
  {0x5208, 0x08},
  {0x5209, 0x4E},
};
static struct msm_camera_i2c_reg_conf ovm7695_daylight[] =
{
  {0x5200, 0x20}, 
  {0x5204, 0x05},
  {0x5205, 0x7B},
  {0x5206, 0x04},
  {0x5207, 0x00},
  {0x5208, 0x05},
  {0x5209, 0x15},
};
static struct msm_camera_i2c_reg_conf ovm7695_cloudy_daylight[] =
{
  {0x5200, 0x20}, 
  {0x5204, 0x06},
  {0x5205, 0x00},
  {0x5206, 0x04},
  {0x5207, 0x00},
  {0x5208, 0x04},
  {0x5209, 0x80},
};
static struct msm_camera_i2c_reg_conf ovm7695_wb_off[] =
{
  {0x5200, 0x00}, 
  {0x520a, 0x74},
  {0x520b, 0x64},
  {0x520c, 0xd4},
};

int ovm7695_update_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array;
  int conf_size = 0, rc;
  char *name;
  if(ovm7695_white_balance_setting == setting)
    return 0;
  if(setting == MSM_CAMERA_WB_MODE_INCANDESCENT)
  {
    conf_array = ovm7695_incandscent;
    conf_size = ARRAY_SIZE(ovm7695_incandscent);
    name = "INCANDESCENT";
  }
  else if(setting == MSM_CAMERA_WB_MODE_FLUORESCENT)
  {
    conf_array = ovm7695_fluorescent;
    conf_size = ARRAY_SIZE(ovm7695_fluorescent);
    name = "FLUORESCENT";
  }
  else if(setting == MSM_CAMERA_WB_MODE_DAYLIGHT)
  {
    conf_array = ovm7695_daylight;
    conf_size = ARRAY_SIZE(ovm7695_daylight);
    name = "DAYLIGHT";
  }
  else if(setting == MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT)
  {
    conf_array = ovm7695_cloudy_daylight;
    conf_size = ARRAY_SIZE(ovm7695_cloudy_daylight);
    name = "CLOUDY_DAYLIGHT";
  }
  else if(setting == MSM_CAMERA_WB_MODE_OFF)
  {
    conf_array = ovm7695_wb_off;
    conf_size = ARRAY_SIZE(ovm7695_wb_off);
    name = "OFF";
  }
  else  
  {
    conf_array = ovm7695_auto_wb;
    conf_size = ARRAY_SIZE(ovm7695_auto_wb);
    name = "AUTO";
    setting = MSM_CAMERA_WB_MODE_AUTO;
  }
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);
  MSG3("## WHITE_BALANCE = %2d, %s",setting,name);
  ovm7695_white_balance_setting = setting;
  return rc;
};




static struct msm_camera_i2c_reg_conf ovm7695_exp_n2[] =
{

  
  {0x3a0f, 0x38}, 
  {0x3a10, 0x30},
  {0x3a11, 0x80},
  {0x3a1b, 0x3A},
  {0x3a1e, 0x2E},
  {0x3a1f, 0x8},
};
static struct msm_camera_i2c_reg_conf ovm7695_exp_n1[] =
{

   
  {0x3a0f, 0x40}, 
  {0x3a10, 0x38},
  {0x3a11, 0x88},
  {0x3a1b, 0x42},
  {0x3a1e, 0x36},
  {0x3a1f, 0x10},
};
static struct msm_camera_i2c_reg_conf ovm7695_exp_d[] =
{

  
  {0x3a0f, 0x48}, 
  {0x3a10, 0x40}, 
  {0x3a11, 0x90}, 
  {0x3a1b, 0x4A}, 
  {0x3a1e, 0x3E}, 
  {0x3a1f, 0x18}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_exp_p1[] =
{

  
  {0x3a0f, 0x50}, 
  {0x3a10, 0x48},
  {0x3a11, 0x98},
  {0x3a1b, 0x52},
  {0x3a1e, 0x46},
  {0x3a1f, 0x20},
};
static struct msm_camera_i2c_reg_conf ovm7695_exp_p2[] =
{

  
  {0x3a0f, 0x58}, 
  {0x3a10, 0x50},
  {0x3a11, 0xa0},
  {0x3a1b, 0x5a},
  {0x3a1e, 0x4e},
  {0x3a1f, 0x28},
};
int ovm7695_update_exposure(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array;
  int conf_size = 0, rc;
  if(ovm7695_exposure_setting == setting)
    return 0;
  
  if(setting <= -4)       {conf_array = ovm7695_exp_n2;  conf_size = ARRAY_SIZE(ovm7695_exp_n2); }
  else if(setting <= -2)  {conf_array = ovm7695_exp_n1;  conf_size = ARRAY_SIZE(ovm7695_exp_n1); }
  else if(setting >= 4)   {conf_array = ovm7695_exp_p2;  conf_size = ARRAY_SIZE(ovm7695_exp_p2); }
  else if(setting >= 2)   {conf_array = ovm7695_exp_p1;  conf_size = ARRAY_SIZE(ovm7695_exp_p1); }
  else                    {conf_array = ovm7695_exp_d;   conf_size = ARRAY_SIZE(ovm7695_exp_d); }
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);
  MSG3("## EXPOSURE      = %2d",setting);
  ovm7695_exposure_setting = setting;
  return rc;
};




static struct msm_camera_i2c_reg_conf ovm7695_saturation_0[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_1[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_2[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_3[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_4[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_5[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_6[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_7[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_8[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_9[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_saturation_10[] =
{

  
  {0x5001, 0x3f}, 
  {0x5803, 0x38}, 
  {0x5804, 0x38}, 
  {0x5800, 0x02}, 
  {0x580b, 0x00}, 
};
int ovm7695_update_saturation(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array;
  int conf_size = 0, rc;
  if(ovm7695_saturation_setting == setting)
    return 0;
  
  if(setting <= 0)        {conf_array = ovm7695_saturation_0;  conf_size = ARRAY_SIZE(ovm7695_saturation_0); }
  else if(setting == 1)   {conf_array = ovm7695_saturation_1;  conf_size = ARRAY_SIZE(ovm7695_saturation_1); }
  else if(setting == 2)   {conf_array = ovm7695_saturation_2;  conf_size = ARRAY_SIZE(ovm7695_saturation_2); }
  else if(setting == 3)   {conf_array = ovm7695_saturation_3;  conf_size = ARRAY_SIZE(ovm7695_saturation_3); }
  else if(setting == 4)   {conf_array = ovm7695_saturation_4;  conf_size = ARRAY_SIZE(ovm7695_saturation_4); }
  else if(setting == 5)   {conf_array = ovm7695_saturation_5;  conf_size = ARRAY_SIZE(ovm7695_saturation_5); }
  else if(setting == 6)   {conf_array = ovm7695_saturation_6;  conf_size = ARRAY_SIZE(ovm7695_saturation_6); }
  else if(setting == 7)   {conf_array = ovm7695_saturation_7;  conf_size = ARRAY_SIZE(ovm7695_saturation_7); }
  else if(setting == 8)   {conf_array = ovm7695_saturation_8;  conf_size = ARRAY_SIZE(ovm7695_saturation_8); }
  else if(setting == 9)   {conf_array = ovm7695_saturation_9;  conf_size = ARRAY_SIZE(ovm7695_saturation_9); }
  else                    {conf_array = ovm7695_saturation_10; conf_size = ARRAY_SIZE(ovm7695_saturation_10); }
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);
  MSG3("## SATURATION      = %2d",setting);
  ovm7695_saturation_setting = setting;
  return rc;
};




static struct msm_camera_i2c_reg_conf ovm7695_contrast_0[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x34},
  {0x5805, 0x24},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_1[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x30},
  {0x5805, 0x20},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_2[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x2c},
  {0x5805, 0x1c},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_3[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x28},
  {0x5805, 0x18},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_4[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x24},
  {0x5805, 0x10},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_5[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x20}, 
  {0x5805, 0x00}, 
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_6[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x1c},
  {0x5805, 0x1c},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_7[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x18},
  {0x5805, 0x18},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_8[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x14},
  {0x5805, 0x14},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_9[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x10},
  {0x5805, 0x10},
  {0x5808, 0x00}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_contrast_10[] =
{
  
  {0x5001, 0x3f}, 
  {0x5800, 0x06}, 
  {0x580b, 0x02}, 
  {0x5806, 0x08},
  {0x5805, 0x08},
  {0x5808, 0x00}, 
};
int ovm7695_update_contrast(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array;
  int conf_size = 0, rc;
  if(ovm7695_contrast_setting == setting)
    return 0;
  
  if(setting <= 0)        {conf_array = ovm7695_contrast_0;  conf_size = ARRAY_SIZE(ovm7695_contrast_0); }
  else if(setting == 1)   {conf_array = ovm7695_contrast_1;  conf_size = ARRAY_SIZE(ovm7695_contrast_1); }
  else if(setting == 2)   {conf_array = ovm7695_contrast_2;  conf_size = ARRAY_SIZE(ovm7695_contrast_2); }
  else if(setting == 3)   {conf_array = ovm7695_contrast_3;  conf_size = ARRAY_SIZE(ovm7695_contrast_3); }
  else if(setting == 4)   {conf_array = ovm7695_contrast_4;  conf_size = ARRAY_SIZE(ovm7695_contrast_4); }
  else if(setting == 5)   {conf_array = ovm7695_contrast_5;  conf_size = ARRAY_SIZE(ovm7695_contrast_5); }
  else if(setting == 6)   {conf_array = ovm7695_contrast_6;  conf_size = ARRAY_SIZE(ovm7695_contrast_6); }
  else if(setting == 7)   {conf_array = ovm7695_contrast_7;  conf_size = ARRAY_SIZE(ovm7695_contrast_7); }
  else if(setting == 8)   {conf_array = ovm7695_contrast_8;  conf_size = ARRAY_SIZE(ovm7695_contrast_8); }
  else if(setting == 9)   {conf_array = ovm7695_contrast_9;  conf_size = ARRAY_SIZE(ovm7695_contrast_9); }
  else                    {conf_array = ovm7695_contrast_10; conf_size = ARRAY_SIZE(ovm7695_contrast_10); }
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);
  MSG3("## CONTRAST      = %2d",setting);
  ovm7695_contrast_setting = setting;
  return rc;
};




static struct msm_camera_i2c_reg_conf ovm7695_sharpen_manual_mode[] =
{
  {0x5508, 0x40, MSM_CAMERA_I2C_SET_BYTE_MASK},
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_0[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_1[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_2[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_3[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_4[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_5[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_6[] =
{
  {0x5502, 0x10}, 
};
static struct msm_camera_i2c_reg_conf ovm7695_sharpness_7[] =
{
  {0x5502, 0x10}, 
};
int ovm7695_update_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array, *shrp_man_en;
  int conf_size = 0, shrp_man_en_size = 0, rc;
    uint16_t value = 0;

  if(ovm7695_sharpness_setting == setting)
    return 0;

#ifdef DEBUG
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            0x5508,
            &value, MSM_CAMERA_I2C_BYTE_DATA);
    pr_err("@@@amy, before before read sharpness value: %x\n", value);
#endif

  shrp_man_en = ovm7695_sharpen_manual_mode;
  shrp_man_en_size = ARRAY_SIZE(ovm7695_sharpen_manual_mode);
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    shrp_man_en, shrp_man_en_size, MSM_CAMERA_I2C_BYTE_DATA);

#ifdef DEBUG
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            0x5508,
            &value, MSM_CAMERA_I2C_BYTE_DATA);
    pr_err("@@@amy, after read sharpness value: %x\n", value);
#endif

  
  if(setting <= 0)        {conf_array = ovm7695_sharpness_0;  conf_size = ARRAY_SIZE(ovm7695_sharpness_0); }
  else if(setting <= 6)   {conf_array = ovm7695_sharpness_1;  conf_size = ARRAY_SIZE(ovm7695_sharpness_1); }
  else if(setting <= 12)  {conf_array = ovm7695_sharpness_2;  conf_size = ARRAY_SIZE(ovm7695_sharpness_2); }
  else if(setting <= 18)  {conf_array = ovm7695_sharpness_3;  conf_size = ARRAY_SIZE(ovm7695_sharpness_3); }
  else if(setting <= 24)  {conf_array = ovm7695_sharpness_4;  conf_size = ARRAY_SIZE(ovm7695_sharpness_4); }
  else if(setting <= 30)  {conf_array = ovm7695_sharpness_5;  conf_size = ARRAY_SIZE(ovm7695_sharpness_5); }
  else if(setting <= 36)  {conf_array = ovm7695_sharpness_6;  conf_size = ARRAY_SIZE(ovm7695_sharpness_6); }
  else                    {conf_array = ovm7695_sharpness_7;  conf_size = ARRAY_SIZE(ovm7695_sharpness_7); }
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);

#if 1
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            0x5502,
            &value, MSM_CAMERA_I2C_BYTE_DATA);
    pr_err("@@@amy, sharpness 0x5502 value: %x\n", value);
#endif

  MSG3("## SHARPNESS      = %2d",setting);
  ovm7695_sharpness_setting = setting;
  return rc;
};




extern int front_cam_exposure_time; 
extern int front_cam_iso_speed;     
void ovm7695_read_iso_exposure_time(struct msm_sensor_ctrl_t *s_ctrl)
{
  int rc1 = 0;
  int iso, gain16 = 0;
  uint16_t r350a;

  
  rc1 = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,
    0x350A, &r350a, MSM_CAMERA_I2C_WORD_DATA);
  if(rc1 < 0) 
  {
    iso = 100;
  }
  else
  {
    gain16 = r350a & 0x1f;
    iso = gain16/16* 100;
    if(iso <= 0)
      iso = 100;
  }
  MSG3("## gain16=%03X, iso=%4d", gain16,iso);
  front_cam_iso_speed = iso;
}




static struct msm_camera_i2c_reg_conf ovm7695_gain_auto[] =
{
  {0x3503, 0x00},
  {0x3a18, 0x00},
  {0x3a19, 0xf8},
};
static struct msm_camera_i2c_reg_conf ovm7695_gain_2x[] =
{
  {0x3503, 0x02},
  {0x3a18, 0x00}, 
  {0x3a19, 0x10},
};
static struct msm_camera_i2c_reg_conf ovm7695_gain_4x[] =
{
  {0x3503, 0x02},
  {0x3a18, 0x00}, 
  {0x3a19, 0x30},
};
static struct msm_camera_i2c_reg_conf ovm7695_gain_8x[] =
{
  {0x3503, 0x02},
  {0x3a18, 0x00}, 
  {0x3a19, 0x70},
};
static struct msm_camera_i2c_reg_conf ovm7695_gain_16x[] =
{
  {0x3503, 0x02},
  {0x3a18, 0x00}, 
  {0x3a19, 0xf0},
};
static struct msm_camera_i2c_reg_conf ovm7695_gain_32x[] =
{
  {0x3503, 0x02},
  {0x3a18, 0x01}, 
  {0x3a19, 0xf0},
};
static struct msm_camera_i2c_reg_conf ovm7695_gain_64x[] =
{
  {0x3503, 0x02},
  {0x3a18, 0x01}, 
  {0x3a19, 0xff},
};
int ovm7695_update_gain(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array;
  int conf_size = 0, rc;
  if(ovm7695_gain_setting == setting)
    return 0;
  
  if(setting <= 0)       {conf_array = ovm7695_gain_auto;  conf_size = ARRAY_SIZE(ovm7695_gain_auto); }
  else if(setting == 1)  {conf_array = ovm7695_gain_2x;    conf_size = ARRAY_SIZE(ovm7695_gain_2x);   }
  else if(setting == 2)   {conf_array = ovm7695_gain_4x;    conf_size = ARRAY_SIZE(ovm7695_gain_4x);  }
  else if(setting == 3)   {conf_array = ovm7695_gain_8x;    conf_size = ARRAY_SIZE(ovm7695_gain_8x);  }
  else if(setting == 4)   {conf_array = ovm7695_gain_16x;   conf_size = ARRAY_SIZE(ovm7695_gain_16x); }
  else if(setting == 5)   {conf_array = ovm7695_gain_32x;   conf_size = ARRAY_SIZE(ovm7695_gain_32x); }
  else                    {conf_array = ovm7695_gain_64x;   conf_size = ARRAY_SIZE(ovm7695_gain_64x); }

  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);
  MSG3("## ISO/GAIN      = %2d",setting);
  ovm7695_gain_setting = setting;
  return rc;
};

#if 1



static struct msm_camera_i2c_reg_conf ovm7695_power_line_off[] =
{
  {0x5002, 0x48},
};
static struct msm_camera_i2c_reg_conf ovm7695_power_line_60hz[] =
{
  {0x5002, 0x48},
};
static struct msm_camera_i2c_reg_conf ovm7695_power_line_50hz[] =
{
  {0x5002, 0x4a},
};
static struct msm_camera_i2c_reg_conf ovm7695_power_line_auto[] =
{
  {0x5002, 0x48},
};
int ovm7695_update_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int setting)
{
  struct msm_camera_i2c_reg_conf *conf_array;
  int conf_size = 0, rc;
  char *name;
  if(ovm7695_antibanding_setting == setting)
    return 0;
  if(setting == 0)      
  {
    conf_array = ovm7695_power_line_off;
    conf_size = ARRAY_SIZE(ovm7695_power_line_off);
    name = "OFF";
  }
  else if(setting == 1) 
  {
    conf_array = ovm7695_power_line_60hz;
    conf_size = ARRAY_SIZE(ovm7695_power_line_60hz);
    name = "60Hz";
  }
  else if(setting == 2) 
  {
    conf_array = ovm7695_power_line_50hz;
    conf_size = ARRAY_SIZE(ovm7695_power_line_50hz);
    name = "50Hz";
  }
  else                  
  {
    conf_array = ovm7695_power_line_auto;
    conf_size = ARRAY_SIZE(ovm7695_power_line_auto);
    name = "AUTO";
    setting = 3;
  }
  rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
    conf_array,	conf_size, MSM_CAMERA_I2C_BYTE_DATA);
  
  MSG3("## BAND_STOP     = %2d, %s",setting,name);
  ovm7695_antibanding_setting = setting;
  return rc;
};
#endif

static int32_t msm_ovm7695_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ovm7695_s_ctrl);
}

static const struct i2c_device_id ovm7695_i2c_id[] = {
	{OVM7695_SENSOR_NAME, (kernel_ulong_t)&ovm7695_s_ctrl},
	{ }
};

static struct i2c_driver ovm7695_i2c_driver = {
	.id_table = ovm7695_i2c_id,
	.probe  = msm_ovm7695_i2c_probe,
	.driver = {
		.name = OVM7695_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ovm7695_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ovm7695_dt_match[] = {
	{.compatible = "qcom,ovm7695", .data = &ovm7695_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ovm7695_dt_match);

static struct platform_driver ovm7695_platform_driver = {
	.driver = {
		.name = "qcom,ovm7695",
		.owner = THIS_MODULE,
		.of_match_table = ovm7695_dt_match,
	},
};

static int32_t ovm7695_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(ovm7695_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ovm7695_init_module(void)
{
	int32_t rc = 0;

	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ovm7695_platform_driver,
		ovm7695_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&ovm7695_i2c_driver);
}

static void __exit ovm7695_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ovm7695_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ovm7695_s_ctrl);
		platform_driver_unregister(&ovm7695_platform_driver);
	} else
		i2c_del_driver(&ovm7695_i2c_driver);
	return;
}

int32_t ovm7695_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	int setting = 0, ret = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		MSG2("## CFG_GET_SENSOR_INFO");
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		MSG2("## CFG_SET_INIT_SETTING");
		
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, ovm7695_recommend_settings,
			ARRAY_SIZE(ovm7695_recommend_settings),
			MSM_CAMERA_I2C_BYTE_DATA);

		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ovm7695_config_stop_settings,
			ARRAY_SIZE(ovm7695_config_stop_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
	case CFG_SET_RESOLUTION:
		MSG2("## CFG_SET_RESOLUTION");
		break;
	case CFG_SET_STOP_STREAM:
		ovm7695_read_iso_exposure_time(s_ctrl);
		MSG2("## CFG_SET_STOP_STREAM");
		break;
	case CFG_SET_START_STREAM:
		MSG2("## CFG_SET_START_STREAM");

		if(ovm7695_just_power_up)
		{
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client,
				ovm7695_config_greenish_settings,
				ARRAY_SIZE(ovm7695_config_greenish_settings),
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			ovm7695_config_start_settings,
			ARRAY_SIZE(ovm7695_config_start_settings),
			MSM_CAMERA_I2C_BYTE_DATA);
		if(ovm7695_just_power_up)
		{
			ovm7695_just_power_up = 0;
			
			if(ovm7695_white_balance_setting == MSM_CAMERA_WB_MODE_AUTO)
				msleep(500);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client,
				ovm7695_config_MWB_settings,
				ARRAY_SIZE(ovm7695_config_MWB_settings),
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		MSG2("## CFG_GET_SENSOR_INIT_PARAMS");
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		MSG2("## CFG_SET_SLAVE_INFO");
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		MSG2("## CFG_WRITE_I2C_ARRAY");
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		MSG2("## CFG_WRITE_I2C_SEQ_ARRAY");
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		MSG2("## CFG_POWER_UP");
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		
		ovm7695_white_balance_setting = -99;
		ovm7695_exposure_setting = -99;
		ovm7695_antibanding_setting = -99;
		ovm7695_saturation_setting = -99;
		ovm7695_contrast_setting = -99;
		ovm7695_sharpness_setting = -99;
		ovm7695_gain_setting = -99;
		ovm7695_just_power_up = 1;
		
		break;

	case CFG_POWER_DOWN:
		MSG2("## CFG_POWER_DOWN");
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		
		ovm7695_white_balance_setting = -99;
		ovm7695_exposure_setting = -99;
		ovm7695_antibanding_setting = -99;
		ovm7695_saturation_setting = -99;
		ovm7695_contrast_setting = -99;
		ovm7695_sharpness_setting = -99;
		ovm7695_gain_setting = -99;
		
		break;

	
	case CFG_SET_ISO: 
		if(copy_from_user(&setting, (void *)cdata->cfg.setting, sizeof(setting)))
			ret = -EFAULT;
		MSG2("## CFG_SET_ISO = %2d, ret = %d",setting,ret);
		ovm7695_update_gain(s_ctrl, setting);
		break;

	case CFG_SET_EXPOSURE_COMPENSATION:
		if(copy_from_user(&setting, (void *)cdata->cfg.setting, sizeof(setting)))
			ret = -EFAULT;
		MSG2("## CFG_SET_EXPOSURE_COMPENSATION = %2d, ret = %d",setting,ret);
		ovm7695_update_exposure(s_ctrl, setting);
		break;

	case CFG_SET_ANTIBANDING:
		if(copy_from_user(&setting, (void *)cdata->cfg.setting, sizeof(setting)))
			ret = -EFAULT;
		MSG2("## CFG_SET_ANTIBANDING = %2d, ret = %d",setting,ret);
		ovm7695_update_antibanding(s_ctrl, setting);
		break;

	case CFG_SET_BESTSHOT_MODE: 
		if(copy_from_user(&setting, (void *)cdata->cfg.setting, sizeof(setting)))
			ret = -EFAULT;
		MSG2("## CFG_SET_BESTSHOT_MODE = %2d, ret = %d",setting,ret);
		break;

	case CFG_SET_EFFECT:  
		if(copy_from_user(&setting, (void *)cdata->cfg.setting, sizeof(setting)))
			ret = -EFAULT;
		MSG2("## CFG_SET_EFFECT = %2d, ret = %d",setting,ret);
		break;

	case CFG_SET_WHITE_BALANCE:
		if(copy_from_user(&setting, (void *)cdata->cfg.setting, sizeof(setting)))
			ret = -EFAULT;
		MSG2("## CFG_SET_WHITE_BALANCE = %2d, ret = %d",setting,ret);
		ovm7695_update_white_balance(s_ctrl, setting);
		break;
	

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		MSG2("## CFG_SET_STOP_STREAM_SETTING");
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
		}
		case CFG_SET_SATURATION: {
			int32_t sat_lev;
			if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			MSG2("## CFG_SET_SATURATION = %2d, ret = %ld",sat_lev,rc);
			ovm7695_update_saturation(s_ctrl, sat_lev);
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
			if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			MSG2("## CFG_SET_CONTRAST = %2d, ret = %ld",con_lev,rc);
			ovm7695_update_contrast(s_ctrl, con_lev);
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);
		break;
		}
		case CFG_SET_SHARPNESS: {
			int32_t shp_lev;
			if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			MSG2("## CFG_SET_SHARPNESS = %2d, ret= %ld",shp_lev,rc);
			ovm7695_update_sharpness(s_ctrl, shp_lev);
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);
		break;
		}
		case CFG_SET_AUTOFOCUS: {
		MSG2("## CFG_SET_AUTOFOCUS");
		
		pr_debug("%s: Setting Auto Focus", __func__);
		break;
		}
		case CFG_CANCEL_AUTOFOCUS: {
		MSG2("## CFG_CANCEL_AUTOFOCUS");
		
		pr_debug("%s: Cancelling Auto Focus", __func__);
		break;
		}
		default:
		MSG2("## default, cfgtype = %d",cdata->cfgtype);
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t ovm7695_sensor_func_tbl = {
	.sensor_config = ovm7695_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t ovm7695_s_ctrl = {
	.sensor_i2c_client = &ovm7695_sensor_i2c_client,
	.power_setting_array.power_setting = ovm7695_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ovm7695_power_setting),
	.msm_sensor_mutex = &ovm7695_mut,
	.sensor_v4l2_subdev_info = ovm7695_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ovm7695_subdev_info),
	.func_tbl = &ovm7695_sensor_func_tbl,
};

module_init(ovm7695_init_module);
module_exit(ovm7695_exit_module);
MODULE_DESCRIPTION("ovm7695 VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");
