/* drivers/input/touchscreen/synaptics_rmi4_touch.c
 *
 * Copyright (c) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */





#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/delay.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <mach/vreg.h>
#include <linux/input/mt.h>
#include <linux/input/synaptics_s3202_touch.h>
#include <linux/input/synaptics_s3202_touch_DS4_3_2.h>
#include <linux/input/synaptics_s3202_touch_DS4_3_6.h>
#include <linux/input/synaptics_s3202_regmap_for_fw1471960.h>
#include "synaptics_s3202_firmware_1200567.h"
#include "synaptics_s3202_firmware_1296077.h"
#include "synaptics_s3202_firmware_1365481.h"
#include "synaptics_s3202_config_JTP_black_31303032.h"
#include "synaptics_s3202_config_JTP_white_31313032.h"
#include "synaptics_s3202_config_TRL_black_52423230.h"
#include "synaptics_s3202_config_4F423134.h"
#include "synaptics_s3202_firmware_1471960.h"
#include "synaptics_s3202_config_TPKSNW_black_53423130.h"
#include <linux/debugfs.h>
#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif 
#define SYNAPTICS_POR_DELAY	100 
#define MAX_TOUCH_MAJOR		15
#include <linux/proc_fs.h>


struct dentry *kernel_debuglevel_dir;
static struct proc_dir_entry *proc_entry; 
static struct proc_dir_entry *proc1_entry; 
static struct proc_dir_entry *proc2_entry; 
static struct proc_dir_entry *proc3_entry; 
static struct proc_dir_entry *proc4_entry; 
#define IS_UNDER_TESTING 0 


#define DBG_REG(fmt, args...) \
	pr_debug("[tp_reg] %s: " fmt "", __func__, args)
#define DBG_TOUCH(fmt, args...) \
	pr_debug("[touch] %s: " fmt "", __func__, args)
#define DBG_KEY(fmt, args...) \
	pr_debug("[key] %s: " fmt "", __func__, args)
	
struct synaptics_tp_t {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct input_dev 	*keyarray_input;
	int                      irq; 
	int                      gpio_irq; 
    int                      gpio_rst; 
    int                      open_count; 
	int                      keyarray_open_count;
	uint                  sensor_max_x;
	uint                  sensor_max_y;
	struct F11_2D_data_t     f11_2d_data;
	int is_suspended;
  struct  mutex            mutex;
	
	
	
	struct regulator *ldo19_regulator; 
	struct regulator *lvs1_regulator; 
	uint8_t			current_page;
	struct synaptics_touch_point_status_t msg[MAX_TS_REPORT_POINTS];
	struct synaptics_cap_key_point_status_t key_msg[MAX_KEY_REPORT_POINTS];

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend ts_early_suspend;
#endif	
	
	struct dentry   *dent;
	uint32_t fw_version;
	uint32_t config_id;
	
	struct synaptics_pdt_register_map_t pdt_map;
	int number_of_rx_electrodes;
	int number_of_tx_electrodes;
	
	int F54_testing_flag;
	uint8_t report_type;
	int report_size;
	uint8_t raw_cap_value[F54_RAW_CAPACITANCE_READ_BYTES];
	uint16_t raw_cap[NUM_OF_TX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint16_t rx_rx_imagearray_7[NUM_OF_TX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint16_t rx_rx_imagearray_17[NUM_OF_RX_TX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint16_t rx_rx_imagearray[NUM_OF_RX_ELECTRODES][NUM_OF_RX_ELECTRODES];
	uint8_t report_type7[RX_RX_7_SIZE];
	uint8_t report_type17[RX_RX_17_SIZE];
	int f54_raw_cap_min_limit; 
	int f54_raw_cap_max_limit; 
	int f54_key_raw_cap_min_limit; 
	int f54_key_raw_cap_max_limit; 
	
	uint16_t f34_bootload_id;
	uint16_t config_block_size;
	uint16_t config_block_count;
	uint16_t config_image_size;
	uint16_t firmware_block_size;
	uint16_t firmware_block_count;
	uint16_t firmware_image_size;
	uint8_t  bootloader_version;
	int program_enable_success;
	
	unsigned char *puData;
	
	unsigned char *puFirmwareData;
	
	
	
	int isKeyAaCriteraDiff; 
	
	uint8_t capKeyTxLinValid[NUM_OF_TX_ELECTRODES];
};

static struct synaptics_tp_t         *g_tp;
static int config_update_result = 1;
static int __devinit touchpad_probe(struct i2c_client *client,  const struct i2c_device_id *);
static int touchpad_setup_gpio(struct synaptics_tp_t *g_tp);
static int __devexit touchpad_remove(struct i2c_client *client);

#define TOUCH_RETRY_COUNT 5
static int touchpad_write_i2c(struct i2c_client *client,
                        uint8_t           regBuf,
                        uint8_t           *dataBuf,
                        uint8_t           dataLen)
{
    int     result = 0;
    uint8_t *buf = NULL;
    int     retryCnt = TOUCH_RETRY_COUNT;
	
    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)buf,
            .len    = 0
        }
    };

    buf = kzalloc(dataLen+sizeof(regBuf), GFP_KERNEL);
    if( NULL == buf )
    {
        pr_err("[touch] %s: alloc memory failed\n", __func__);
        return -EFAULT;
    }

    buf[0] = regBuf;
    memcpy(&buf[1], dataBuf, dataLen);
    msgs[0].buf = buf;
    msgs[0].len = dataLen+1;

    while(retryCnt)
    {
        result = i2c_transfer(client->adapter, msgs, 1);
        if(result != ARRAY_SIZE(msgs))
        {
            pr_err("[touch] %s: write %Xh %d bytes return failure\n", __func__, buf[0], dataLen);
            if(-ETIMEDOUT == result) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        pr_err("[touch] %s: write %Xh %d bytes retry at %d\n", __func__, buf[0], dataLen, TOUCH_RETRY_COUNT-retryCnt);
    kfree( buf );
    return result;
}

static int touchpad_read_i2c(struct i2c_client *client,
                        uint16_t           regBuf,
                        uint8_t           *dataBuf,
                        uint8_t           dataLen)
{
    int     result = 0;
    int     retryCnt = TOUCH_RETRY_COUNT;

    struct  i2c_msg msgs[] = {
        [0] = {
            .addr   = client->addr,
            .flags  = 0,
            .buf    = (void *)&regBuf,
            .len    = 1
        },
        [1] = {
            .addr   = client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)dataBuf,
            .len    = dataLen
        }
    };

    while(retryCnt)
    {
        result = i2c_transfer(client->adapter, msgs, 2);
        if( result != ARRAY_SIZE(msgs) )
        {
            pr_err("[touch] %s: read %Xh %d bytes return failure.\n", __func__, regBuf, dataLen);
            if( -ETIMEDOUT == result ) msleep(10);
            retryCnt--;
        }else {
            result = 0;
            break;
        }
    }

    if( (result == 0) && (retryCnt < TOUCH_RETRY_COUNT) )
        pr_err("[touch] %s: read %Xh %d bytes retry at %d\n", __func__, regBuf, dataLen, TOUCH_RETRY_COUNT-retryCnt);

    return result;
}

static void touchpad_report_capkey(struct synaptics_tp_t *g_tp,uint8_t F1A_0D_data)
{
	
	if (F1A_0D_data & 0x1) {
		g_tp->key_msg[0].key_state = KEY_PRESS;
		DBG_KEY("BACK_KEY[%d] is pressed\n", KEY_BACK);
		input_report_key(g_tp->keyarray_input, KEY_BACK, 1);
	} else {
		g_tp->key_msg[0].key_state = KEY_RELEASE;
		DBG_KEY("BACK_KEY[%d] is released\n", KEY_BACK);
		input_report_key(g_tp->keyarray_input, KEY_BACK, 0);
	}
	if (F1A_0D_data & 0x2) {
		g_tp->key_msg[0].key_state = KEY_PRESS;
		DBG_KEY("HOME_KEY[%d] is pressed\n", KEY_HOMEPAGE);
		input_report_key(g_tp->keyarray_input, KEY_HOMEPAGE, 1);
	} else {
		g_tp->key_msg[0].key_state = KEY_RELEASE;
		DBG_KEY("HOME_KEY[%d] is released\n", KEY_HOMEPAGE);
		input_report_key(g_tp->keyarray_input, KEY_HOMEPAGE, 0);
	}
	if (F1A_0D_data & 0x4) {
		g_tp->key_msg[0].key_state = KEY_PRESS;
		DBG_KEY("MENU_KEY[%d] is pressed\n", KEY_MENU);
		input_report_key(g_tp->keyarray_input, KEY_MENU, 1);
	} else {
		g_tp->key_msg[0].key_state = KEY_RELEASE;
		DBG_KEY("MENU_KEY[%d] is released\n", KEY_MENU);
		input_report_key(g_tp->keyarray_input, KEY_MENU, 0);
	}
	input_sync(g_tp->keyarray_input);
	return;
}

static void touchpad_report_mt_protocol(struct synaptics_tp_t *g_tp)
{
	int i;
	int all_up = 1;
	
	for(i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		if (g_tp->msg[i].coord.z == -1)
			continue;
		if(g_tp->msg[i].prev_state == TS_RELEASE && g_tp->msg[i].state == TS_PRESS) {
			DBG_TOUCH("skip <id=%d, (x,y)=(%d,%d), (z,w)=(%d,%d)>\n",
				i , g_tp->msg[i].coord.x, g_tp->msg[i].coord.y, g_tp->msg[i].coord.z, g_tp->msg[i].coord.wxy);
		} else {
			input_report_abs(g_tp->input, ABS_MT_POSITION_X, g_tp->msg[i].coord.x);
			input_report_abs(g_tp->input, ABS_MT_POSITION_Y, g_tp->msg[i].coord.y);
			if(g_tp->msg[i].coord.z == 0)input_report_abs(g_tp->input, ABS_MT_PRESSURE, 1);
			else input_report_abs(g_tp->input, ABS_MT_PRESSURE, g_tp->msg[i].coord.z);
			DBG_TOUCH("id=%d, (x,y)=(%d,%d), (z,w)=(%d,%d)\n",
				i , g_tp->msg[i].coord.x, g_tp->msg[i].coord.y, g_tp->msg[i].coord.z, g_tp->msg[i].coord.wxy);
			if(g_tp->msg[i].coord.z != 0) all_up = 0;
			input_mt_sync(g_tp->input);
		}
			if (g_tp->msg[i].coord.z == 0)
				g_tp->msg[i].coord.z = -1;
	}
	input_sync(g_tp->input);
	if(all_up) 
	{
		input_mt_sync(g_tp->input);
		input_sync(g_tp->input);
	}
}

static void touchpad_report_coord(struct synaptics_tp_t *g_tp)
{                 
	
	int i;
	uint8_t state;
	
	for (i = 0; i < MAX_TS_REPORT_POINTS; i++) {
		g_tp->msg[i].coord.x = (uint)g_tp->f11_2d_data.x[i];
		g_tp->msg[i].coord.y = (uint)g_tp->f11_2d_data.y[i];
		state = ((g_tp->f11_2d_data.finger_state[i/4] & (0x3 << ((i%4)*2))) >> ((i%4)*2));
			if (state == 0 && g_tp->msg[i].coord.z != -1) {
			g_tp->msg[i].prev_state = g_tp->msg[i].state;
				g_tp->msg[i].state = TS_RELEASE;
				g_tp->msg[i].coord.wxy = (uint)g_tp->f11_2d_data.wxy[i];
				g_tp->msg[i].coord.z = 0;
			} else if(state == 1 || state == 2) {
			g_tp->msg[i].prev_state = g_tp->msg[i].state;
				g_tp->msg[i].state = TS_PRESS;
				g_tp->msg[i].coord.wxy = (uint)g_tp->f11_2d_data.wxy[i];
				g_tp->msg[i].coord.z = (uint)g_tp->f11_2d_data.z[i];
			} else
				if(state) pr_err("[touch] %s: error finger state = 0x%x\n", __func__, state);		
    }
	touchpad_report_mt_protocol(g_tp);
}


static irqreturn_t touchpad_irq_thread(int irq, void *dev_id)
{
	struct i2c_client	*client = g_tp->client;
	struct synaptics_tp_t *g_tp = dev_id;
    int rc = 0;
    uint8_t page_select,inrt_status;
	uint8_t F11_2D_data[28];
	uint8_t F1A_0D_data;
    mutex_lock(&g_tp->mutex);
	disable_irq_nosync(g_tp->irq);	
			
	
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+1, &inrt_status, 1);
	if (rc < 0) {
		pr_err("[touch] Failed to read the interrupt status reg([0x%x]=0x%x), (rc=%d)\n", 
			g_tp->pdt_map.F01_data_base, inrt_status, rc);
		enable_irq(g_tp->irq);
		mutex_unlock(&g_tp->mutex);
		return IRQ_HANDLED;
	}
	
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_data_base+2, &F11_2D_data[0], 28);
	if (rc < 0) {
		pr_err("[touch] Failed to read finger state reg([0x%x]:0x%x), (rc=%d)\n", 
				g_tp->pdt_map.F01_data_base+2, F11_2D_data[0], rc);
		enable_irq(g_tp->irq);
		mutex_unlock(&g_tp->mutex);
		return IRQ_HANDLED;
	}
	
	g_tp->f11_2d_data.finger_state[0] = F11_2D_data[0];
	g_tp->f11_2d_data.finger_state[1] = F11_2D_data[1];
	g_tp->f11_2d_data.x[0] = (F11_2D_data[3] << 4) | (F11_2D_data[5] & 0x0F);
	g_tp->f11_2d_data.y[0] = (F11_2D_data[4] << 4) | (F11_2D_data[5] & 0xF0) >> 4;
	g_tp->f11_2d_data.wxy[0] = F11_2D_data[6];
	g_tp->f11_2d_data.z[0] = F11_2D_data[7];
	g_tp->f11_2d_data.x[1] = (F11_2D_data[8] << 4) | (F11_2D_data[10] & 0x0F);
	g_tp->f11_2d_data.y[1] = (F11_2D_data[9] << 4) | (F11_2D_data[10] & 0xF0) >> 4;
	g_tp->f11_2d_data.wxy[1] = F11_2D_data[11];
	g_tp->f11_2d_data.z[1] = F11_2D_data[12];
	g_tp->f11_2d_data.x[2] = (F11_2D_data[13] << 4) | (F11_2D_data[15] & 0x0F);
	g_tp->f11_2d_data.y[2] = (F11_2D_data[14] << 4) | (F11_2D_data[15] & 0xF0) >> 4;
	g_tp->f11_2d_data.wxy[2] = F11_2D_data[16];
	g_tp->f11_2d_data.z[2] = F11_2D_data[17];
	g_tp->f11_2d_data.x[3] = (F11_2D_data[18] << 4) | (F11_2D_data[20] & 0x0F);
	g_tp->f11_2d_data.y[3] = (F11_2D_data[19] << 4) | (F11_2D_data[20] & 0xF0) >> 4;
	g_tp->f11_2d_data.wxy[3] = F11_2D_data[21];
	g_tp->f11_2d_data.z[3] = F11_2D_data[22];
	g_tp->f11_2d_data.x[4] = (F11_2D_data[23] << 4) | (F11_2D_data[25] & 0x0F);
	g_tp->f11_2d_data.y[4] = (F11_2D_data[24] << 4) | (F11_2D_data[25] & 0xF0) >> 4;
	g_tp->f11_2d_data.wxy[4] = F11_2D_data[26];
	g_tp->f11_2d_data.z[4] = F11_2D_data[27];
		
	if (inrt_status == 0x4 || inrt_status == 0x14 || inrt_status == 0x24) {
		touchpad_report_coord(g_tp);
		} else if (inrt_status == 0x10 || inrt_status == 0x20) { 
		
		page_select = 0x02;
		rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
		if (rc) {
			pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
				PAGE_SELECT_REGISTER, page_select, rc);
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F1A_data_base, &F1A_0D_data, sizeof(F1A_0D_data));
		if (rc) {
			pr_err("[touch] failed to read the F1A_0D_data reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F1A_data_base, F1A_0D_data, rc);
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
		touchpad_report_capkey(g_tp,F1A_0D_data);
		page_select = 0x00;
		rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
		if (rc) {
			pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
				PAGE_SELECT_REGISTER, page_select, rc);
			enable_irq(g_tp->irq);
			mutex_unlock(&g_tp->mutex);
			return IRQ_HANDLED;
		}
	} else {
				
	}
	enable_irq(g_tp->irq);
	mutex_unlock(&g_tp->mutex);
    return IRQ_HANDLED;
}

static int touchpad_keyarray_open(struct input_dev *dev)
{
	int rc = 0;
    
    mutex_lock(&g_tp->mutex);
    if(g_tp->keyarray_open_count == 0)
        g_tp->keyarray_open_count++; 
    mutex_unlock(&g_tp->mutex);
    return rc;
}

static void touchpad_keyarray_close(struct input_dev *dev)
{
	mutex_lock(&g_tp->mutex);
    if(g_tp->keyarray_open_count > 0)
        g_tp->keyarray_open_count--;
    mutex_unlock(&g_tp->mutex);
}

static int keyarray_register_input( struct input_dev **input,
                              struct i2c_client *client )
{
	int rc = 0;
	struct input_dev *input_dev;
  
	input_dev = input_allocate_device();
	if (!input_dev)
	{
		rc = -ENOMEM;
		return rc;
	}

	input_dev->name = SYNAPTICS_CAPKEY_NAME;
	input_dev->phys = "synaptcis_s3202_capkey/event0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &client->dev;
	input_dev->open = touchpad_keyarray_open;
	input_dev->close = touchpad_keyarray_close;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOMEPAGE, input_dev->keybit);
    set_bit(KEY_MENU, input_dev->keybit);
	
	rc = input_register_device(input_dev);
	if (rc) {
		pr_err("[touch] %s: Failed to register keyarray input device.\n", SYNAPTICS_CAPKEY_NAME);
		input_free_device(input_dev);
	}
	else {
		*input = input_dev;
	}
  return rc;
}

static int touchpad_open(struct input_dev *dev)
{
    int rc = 0;
    
    mutex_lock(&g_tp->mutex);
    if(g_tp->open_count == 0)
        g_tp->open_count++;
    mutex_unlock(&g_tp->mutex);
    return rc;
}

static void touchpad_close(struct input_dev *dev)
{
    mutex_lock(&g_tp->mutex);
    if(g_tp->open_count > 0)
        g_tp->open_count--;
    mutex_unlock(&g_tp->mutex);
}


static int touchpad_register_input( struct input_dev **input,
                                    struct synaptics_tp_platform_data_t *pdata,
                                    struct i2c_client *client )
{
    int rc = 0;
    struct input_dev *input_dev;
    int i;
    uint8_t page_select;
	uint8_t value[4];
    i = 0;
	
    input_dev = input_allocate_device();
    if ( !input_dev ) {
        rc = -ENOMEM;
        return rc;
    }
    input_dev->name = SYNAPTICS_TP_NAME;
    input_dev->phys = "synaptcis_s3202_ts/input0";
    input_dev->id.bustype = BUS_I2C;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0002;
    input_dev->id.version = 0x0100;
    input_dev->dev.parent = &client->dev;
    input_dev->open = touchpad_open;
    input_dev->close = touchpad_close;
    input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit); 
	
	
	page_select = 0x00;
	rc = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
	
	
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F11_control_base+6, &value[0], 4);
	if (rc) {
		pr_err("[touch] failed to read the MAX_X_POSITION reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F11_control_base+6, value[0], rc);
		return rc;
	}
	g_tp->sensor_max_x = (value[1] & 0x0F) << 8 | (value[0] & 0xFF);
	g_tp->sensor_max_y = (value[3] & 0x0F) << 8 | (value[2] & 0xFF);
	pr_info("[touch], the maximum touch resolution is x*y(%d,%d).\n", (int)g_tp->sensor_max_x, (int)g_tp->sensor_max_y);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, (int)g_tp->sensor_max_x, 0, 0); 
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, (int)g_tp->sensor_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	
    rc = input_register_device(input_dev);
    if (rc) {
        pr_err("[touch] %s: Failed to register input device\n", SYNAPTICS_TP_NAME);
        input_free_device(input_dev);
    }else {
        *input = input_dev;
    }
    return rc;
}


static int touchpad_pdtscan(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	int ret = 0;
	uint8_t page_select = 0x00;
	uint8_t pdt_properities_value = 0x00;
	uint8_t pdt_addr,pdt_value[PDT_SIZE];
	uint8_t i;
	
	memset(pdt_value,0,PDT_SIZE);
	
	for (i = 0x00; i < NUM_OF_DS4_PAGES; i++) {
		page_select = i;
		
		rc = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, 1);
		if (rc) {
			pr_err("[touch] Failed to write the page select reg([0x%x]=0x%x), (rc=%d)\n",
				PAGE_SELECT_REGISTER, page_select, rc);
			goto out;
		}
		msleep(1);
		
		
		rc = touchpad_read_i2c(g_tp->client, PDT_PROPERTIES_ADDR, &pdt_properities_value, 1);
		if (rc) {
			pr_err("[touch] Failed to read teh PDT properties reg([0x%x]=0x%x), (rc=%d)\n",
				PDT_PROPERTIES_ADDR, pdt_properities_value, rc);
			goto out;
		}
		
		if (pdt_properities_value == 0x40) {
			pr_err("[touch] Failed to support the Page Description Table.\n");
			rc = -EFAULT;
			goto out;
		} else if (pdt_properities_value == 0x00) {
			for(pdt_addr = (PDT_PROPERTIES_ADDR-PDT_SIZE); pdt_addr < 0xFF;  pdt_addr-=PDT_SIZE) {  
				
				rc = touchpad_read_i2c(g_tp->client, pdt_addr, &pdt_value[0], PDT_SIZE);
				if (rc) {
					pr_err("[touch] Failed to read the function regsisters map([0x%x]=0x%x), (rc=%d)\n",
						pdt_addr, pdt_value[0], rc);
					goto out;
				}	
				if (page_select == 0x00) {
					if (pdt_value[5] == 0x00) {
						pr_debug("the reserved register value([0x%x]=0x%x)\n", pdt_addr+5, pdt_value[5]);
						break;
					} else if (pdt_value[5] == PDT_FLASH_F34) {
						g_tp->pdt_map.F34_query_base = pdt_value[0];
						g_tp->pdt_map.F34_command_base = pdt_value[1];
						g_tp->pdt_map.F34_control_base = pdt_value[2];
						g_tp->pdt_map.F34_data_base = pdt_value[3];
						g_tp->pdt_map.F34_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F34_function_exists = pdt_value[5];
						DBG_REG("the F34_query_base reg([0x%x]=0x%x)\n", pdt_addr, g_tp->pdt_map.F34_query_base);
						DBG_REG("the F34_command_base reg([0x%x]=0x%x)\n", pdt_addr+1, g_tp->pdt_map.F34_command_base);
						DBG_REG("the F34_control_base reg([0x%x]=0x%x)\n", pdt_addr+2, g_tp->pdt_map.F34_control_base);
						DBG_REG("the F34_data_base reg[0x%x]=0x%x\n", pdt_addr+3, g_tp->pdt_map.F34_data_base);
						DBG_REG("the F34_version_interrupt_count reg([0x%x]=0x%x)\n", pdt_addr+4, g_tp->pdt_map.F34_version_interrupt_count);
						DBG_REG("the F34_function_exists reg([0x%x]=0x%x)\n", pdt_addr+5, g_tp->pdt_map.F34_function_exists);
					} else if (pdt_value[5] == PDT_RMI_F01) {
						g_tp->pdt_map.F01_query_base = pdt_value[0];
						g_tp->pdt_map.F01_command_base = pdt_value[1];
						g_tp->pdt_map.F01_control_base = pdt_value[2];
						g_tp->pdt_map.F01_data_base = pdt_value[3];
						g_tp->pdt_map.F01_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F01_function_exists = pdt_value[5];
						DBG_REG("the F01_query_base reg([0x%x]=0x%x)\n", pdt_addr, g_tp->pdt_map.F01_query_base);
						DBG_REG("the F01_command_base reg([0x%x]=0x%x)\n", pdt_addr+1, g_tp->pdt_map.F01_command_base);
						DBG_REG("the F01_control_base reg([0x%x]=0x%x)\n", pdt_addr+2, g_tp->pdt_map.F01_control_base );
						DBG_REG("the F01_data_base reg([0x%x]=0x%x)\n", pdt_addr+3, g_tp->pdt_map.F01_data_base);
						DBG_REG("the F01_version_interrupt_count reg([0x%x]=0x%x)\n", pdt_addr+4, g_tp->pdt_map.F01_version_interrupt_count);
						DBG_REG("the F01_function_exists reg([0x%x]=0x%x)\n", pdt_addr+5, g_tp->pdt_map.F01_function_exists);
					} else if (pdt_value[5] == PDT_2D_DATA_F11) {
						g_tp->pdt_map.F11_query_base = pdt_value[0];
						g_tp->pdt_map.F11_command_base = pdt_value[1];
						g_tp->pdt_map.F11_control_base = pdt_value[2];
						g_tp->pdt_map.F11_data_base = pdt_value[3];
						g_tp->pdt_map.F11_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F11_function_exists = pdt_value[5];
						DBG_REG("the F11_query_base reg([0x%x]=0x%x)\n", pdt_addr, g_tp->pdt_map.F11_query_base);
						DBG_REG("the F11_command_base reg([0x%x]=0x%x)\n", pdt_addr+1, g_tp->pdt_map.F11_command_base);
						DBG_REG("the F11_control_base reg([0x%x]=0x%x)\n", pdt_addr+2, g_tp->pdt_map.F11_control_base);
						DBG_REG("the F11_data_base reg([0x%x]=0x%x)\n", pdt_addr+3, g_tp->pdt_map.F11_data_base);
						DBG_REG("the F11_version_interrupt_count reg([0x%x]=0x%x)\n", pdt_addr+4, g_tp->pdt_map.F11_version_interrupt_count);
						DBG_REG("the F11_function_exists reg([0x%x]=0x%x)\n", pdt_addr+5, g_tp->pdt_map.F11_function_exists);
					}
				} else if (page_select == 0x01)	{
					if (pdt_value[5] == 0x00) {
						break;
					} else if (pdt_value[5] == PDT_ANALOG_F54) {
						g_tp->pdt_map.F54_query_base = pdt_value[0];
						g_tp->pdt_map.F54_command_base = pdt_value[1];
						g_tp->pdt_map.F54_control_base = pdt_value[2];
						g_tp->pdt_map.F54_data_base = pdt_value[3];
						g_tp->pdt_map.F54_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F54_function_exists = pdt_value[5];
						DBG_REG("the F54_query_base reg([0x%x]=0x%x)\n", pdt_addr, g_tp->pdt_map.F54_query_base);
						DBG_REG("the F54_command_base reg([0x%x]=0x%x)\n", pdt_addr+1, g_tp->pdt_map.F54_command_base);
						DBG_REG("the F54_control_base reg([0x%x]=0x%x)\n", pdt_addr+2, g_tp->pdt_map.F54_control_base);
						DBG_REG("the F54_data_base reg([0x%x]=0x%x)", pdt_addr+3, g_tp->pdt_map.F54_data_base);
						DBG_REG("the F54_version_interrupt_count reg([0x%x]=0x%x)\n", pdt_addr+4, g_tp->pdt_map.F54_version_interrupt_count);
						DBG_REG("the F54_function_exists reg([0x%x]=0x%x)\n", pdt_addr+5, g_tp->pdt_map.F54_function_exists);
					}	
				} else if (page_select == 0x02)	{
					if (pdt_value[5] == 0x00) {
						break;
					} else if (pdt_value[5] == PDT_LED_F31) {
						g_tp->pdt_map.F31_query_base = pdt_value[0];
						g_tp->pdt_map.F31_command_base = pdt_value[1];
						g_tp->pdt_map.F31_control_base = pdt_value[2];
						g_tp->pdt_map.F31_data_base = pdt_value[3];
						g_tp->pdt_map.F31_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F31_function_exists = pdt_value[5];
						DBG_REG("the F31_query_base reg([0x%x]=0x%x)\n", pdt_addr, g_tp->pdt_map.F31_query_base);
						DBG_REG("the F31_command_base reg([0x%x]=0x%x)\n", pdt_addr+1, g_tp->pdt_map.F31_command_base);
						DBG_REG("the F31_control_base reg([0x%x]=0x%x)n", pdt_addr+2, g_tp->pdt_map.F31_control_base);
						DBG_REG("the F31_data_base reg([0x%x]=0x%x)\n", pdt_addr+3, g_tp->pdt_map.F31_data_base);
						DBG_REG("the F31_version_interrupt_count reg([0x%x]=0x%x)\n", pdt_addr+4, g_tp->pdt_map.F31_version_interrupt_count);
						DBG_REG("the F31_function_exists reg([0x%x]=0x%x)\n", pdt_addr+5, g_tp->pdt_map.F31_function_exists);
					} else if (pdt_value[5] == PDT_0D_F1A) {
						g_tp->pdt_map.F1A_query_base = pdt_value[0];
						g_tp->pdt_map.F1A_command_base = pdt_value[1];
						g_tp->pdt_map.F1A_control_base = pdt_value[2];
						g_tp->pdt_map.F1A_data_base = pdt_value[3];
						g_tp->pdt_map.F1A_version_interrupt_count = pdt_value[4];
						g_tp->pdt_map.F1A_function_exists = pdt_value[5];
						DBG_REG("the F1A_query_base reg([0x%x]=0x%x)\n", pdt_addr, g_tp->pdt_map.F1A_query_base);
						DBG_REG("the F1A_command_base reg([0x%x]=0x%x)\n", pdt_addr+1, g_tp->pdt_map.F1A_command_base);
						DBG_REG("the FA1_control_base reg([0x%x]=0x%x)\n", pdt_addr+2, g_tp->pdt_map.F1A_control_base );
						DBG_REG("the F1A_data_base reg([0x%x]=0x%x)\n", pdt_addr+3, g_tp->pdt_map.F1A_data_base);
						DBG_REG("the F1A_version_interrupt_count reg([0x%x]=0x%x)\n", pdt_addr+4, g_tp->pdt_map.F1A_version_interrupt_count);
						DBG_REG("the F1A_function_exists reg([0x%x]=0x%x)\n", pdt_addr+5, g_tp->pdt_map.F1A_function_exists);
					}	
				} else {
					
					pr_err("[touch] the register map table out of range!!\n");
				}
			}	
		}
	}
out:
	
	page_select = 0x00;
	ret = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select));
	if (ret) {
		pr_err("[touch] failed to write the pdt reg([0x%x]=0x%x), (ret=%d)\n",
			PAGE_SELECT_REGISTER, page_select, ret);
		return ret;
	}
	msleep(1);
    return rc;
}


 static int SynaWriteBootloadID(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t bootid[2];
	uint8_t f34_block_data_addr;
	
	f34_block_data_addr = g_tp->pdt_map.F34_data_base + f34_flash_data02_00;
	bootid[0] = g_tp->f34_bootload_id & 0x00FF;
	bootid[1] = (g_tp->f34_bootload_id & 0xFF00) >> 8;
	
	rc = touchpad_write_i2c(g_tp->client, f34_block_data_addr, &bootid[0], 2);
	if (rc) {
		pr_err("[touch] failed to write the F34_Flash Block Data([0x%x]=0x%x), (rc=%d)\n",
			f34_block_data_addr, bootid[0], rc);
		return rc;
	}
	
	rc = touchpad_read_i2c(g_tp->client, f34_block_data_addr, &bootid[0], 2);
	if (rc) {
		pr_err("[touch] failed to read the F34_Flash Block Data([0x%x]=0x%x), (rc=%d)\n",
			f34_block_data_addr, bootid[0], rc);
		return rc;
	}
	return rc;
}


static int SynaReadBootloadID(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t bootid[2];
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F34_query_base, &bootid[0], 2);
	if (rc) {
		pr_err("[touch] failed to read the BootloaderID reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F34_query_base, bootid[0], rc);
		return rc;
	}
	g_tp->f34_bootload_id = bootid[1] << 8 | bootid[0];
	pr_info("[touch] the BootloadID is %d\n", (int)g_tp->f34_bootload_id);
	return rc;
}


static int SynaEnableFlashing(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr, program_enable;
	uint8_t irq_status;
	uint8_t f34_flash_cmd;
	int i;

	pr_info("[touch] %s: start.\n", __func__);
	
	rc = SynaReadBootloadID(g_tp);
	if (rc) {
		pr_err("[touch] failed to read the Bootload ID, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	rc = SynaWriteBootloadID(g_tp);
	if (rc) {
		pr_err("[touch] failed to write the Bootload ID, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	
	
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to read the F34_Flash_Control reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	
	
	f34_flash_cmd |= 0x0f;
	rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to write the F34_flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	msleep(500); 
		
	for(i=0;;i++)
	{
		
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
		if (rc) {
			pr_err("[touch] failed to read the F01_interrupt_status reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F01_data_base+1, irq_status, rc);
			return rc;
		}
		if (irq_status & 0x1)  break;
		if(i == 500) 
		{
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	
	
	rc = touchpad_pdtscan(g_tp);
	if (rc < 0) {
			pr_err("[touch] Failed to reads the RMI5 register map. (rc=%d)\n", rc);
			return rc;
	}
	
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
		pr_err("[touch] Failed to read the program cmd of F34_flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, program_enable, rc);
		return rc;
	}
	
	if (program_enable == 0x80){
		pr_info("[touch] %s: enabled flash programming successfully!!\n", __func__);
		g_tp->program_enable_success = 1;
		rc = 0;
	}else {
		pr_err("[touch] %s: failed to enable flash programming!!\n", __func__);
		g_tp->program_enable_success = 0;
		rc = -1;
	}
	return rc;
}


static int SynaReadFirmwareInfo(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_blocksize_addr,f34_blocksize_value[2];
	uint8_t f34_blockcount_addr,f34_blockcount_value[2];
	
	
	f34_blocksize_addr = g_tp->pdt_map.F34_query_base + f34_flash_query03;
	rc = touchpad_read_i2c(g_tp->client, f34_blocksize_addr, &f34_blocksize_value[0], 2);
	if (rc) {
		pr_err("[touch] failed to read the firmware_block_size of F34 reg([0x%x]=0x%x), (rc=%d)\n",
			f34_blocksize_addr, f34_blocksize_value[0], rc);
		return rc;
	}
	g_tp->firmware_block_size = f34_blocksize_value[1] << 8 | f34_blocksize_value[0];
	
	
	f34_blockcount_addr = g_tp->pdt_map.F34_query_base + f34_flash_query05;
	rc = touchpad_read_i2c(g_tp->client, f34_blockcount_addr, &f34_blockcount_value[0], 2);
	if (rc) {
		pr_err("[touch] failed to read the firmware_block_count of F34 reg([0x%x]=0x%x), (rc=%d)\n",
			f34_blockcount_addr, f34_blockcount_value[0], rc);
		return rc;
	}
	g_tp->firmware_block_count = f34_blockcount_value[1] << 8 | f34_blockcount_value[0];
	
	g_tp->firmware_image_size = g_tp->firmware_block_count * g_tp->firmware_block_size;
	return rc;
}


static int SynaReadConfigInfo(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_blocksize_addr,f34_blocksize_value[2];
	uint8_t f34_blockcount_addr,f34_blockcount_value[2];
	
	
	f34_blocksize_addr = g_tp->pdt_map.F34_query_base + f34_flash_query03;
	rc = touchpad_read_i2c(g_tp->client, f34_blocksize_addr, &f34_blocksize_value[0], 2);
	if (rc) {
		pr_err("[touch] failed to read the config_block_size of F34 reg([0x%x]=0x%x), (rc=%d)\n",
			f34_blocksize_addr, f34_blocksize_value[0], rc);
		return rc;
	}
	g_tp->config_block_size = f34_blocksize_value[1] << 8 | f34_blocksize_value[0];
	
	
	f34_blockcount_addr = g_tp->pdt_map.F34_query_base + f34_flash_query07;
	rc = touchpad_read_i2c(g_tp->client, f34_blockcount_addr, &f34_blockcount_value[0], 2);
	if (rc) {
		pr_err("[touch] failed to read the config_block_count of F34 reg([0x%x]=0x%x), (rc=%d)\n",
			f34_blockcount_addr, f34_blockcount_value[0], rc);
		return rc;
	}
	g_tp->config_block_count = f34_blockcount_value[1] << 8 | f34_blockcount_value[0];
	
	g_tp->config_image_size = g_tp->config_block_count * g_tp->config_block_size;
	return rc;
}


static int SynaProgramConfiguration(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t  blockNum_value[2];
	uint16_t blockNum;
	uint8_t f34_flash_cmd,f34_flash_cmd_addr;
    uint8_t irq_status;
	int i;
	uint8_t program_enable;
	
	pr_info("[touch] %s: start.\n", __func__);
	for (blockNum = 0x000; blockNum < g_tp->config_block_count; blockNum++)
	{
	    blockNum_value[0] = blockNum & 0x00ff;
		blockNum_value[1] = (blockNum & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base, &blockNum_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write the F34_blockNum reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F34_data_base, blockNum_value[0], rc);
			return rc;
		}
		
		
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base+2, g_tp->puData, (int)g_tp->config_block_size);
		if (rc) {
			pr_err("[touch] failed to write the F34_block_data([0x%x]=0x%s), (rc=%d)\n",
				g_tp->pdt_map.F34_data_base+2, g_tp->puData, rc);
			return rc;
		}
		g_tp->puData += (int)g_tp->config_block_size;
		
		f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			pr_err("[touch] failed to read the F34_Flash_Control reg([0x%x]=0x%x), (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		
		f34_flash_cmd = (f34_flash_cmd & 0xF0) | 0x06;
		rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			pr_err("[touch] failed to write the f34_enable_flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		msleep(10); 
		for(i=0;;i++)
		{
			
			rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
			if (rc) {
				pr_err("[touch] failed to read the F01_interrupt_status reg([0x%x]=0x%x), (rc=%d)\n",
					g_tp->pdt_map.F01_data_base+1, irq_status, rc);
				return rc;
			}
			if (irq_status & 0x1)  break;
			if(i == 500) 
			{
				rc = -1;
				return rc;
			}
			msleep(10);
		}
		
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
		if (rc) {
			pr_err("[touch] failed to read the program cmd pf F34 flash reg([0x%x]=0x%x), (rc=%d)\n",
				f34_flash_cmd_addr, program_enable, rc);
			return rc;
		}
		if (program_enable == 0x80){
			pr_info("[touch] %s: enabled flash programming successfully!!\n", __func__);
			rc = 0;
		}else {
			pr_err("[touch] %s: failed to enable flash programming!!\n", __func__);
			rc = -1;
			break;
		}
	}
	return rc;
}


static int SynaFinalizeReflash(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,f34_flash_cmd;
	uint8_t f01_reset_cmd = 0x0;
	uint8_t f01_device_status = 0x0;
	int i;
	
	pr_info("[touch] %s: start.\n", __func__);
	
	
	f01_reset_cmd = 0x1;
	rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F01_command_base, &f01_reset_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to write the F01_reset_cmd reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_command_base, f01_reset_cmd, rc);
		return rc;
	}
	else
	{
		pr_info("[touch] %s: send the reset cmd successfully!!\n", __func__);
	}
	msleep(500); 
	for(i=0;;i++)
	{
		
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base, &f01_device_status, 1);
		if (rc) {
			pr_err("[touch] failed to read the device_status of F01 reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F01_data_base, f01_device_status, rc);
			return rc;
		}
		
		
		
		f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			pr_err("[touch] failed to read the f34_flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_data_base, f01_device_status, rc);
				return rc;
		}
		if( (f01_device_status & 0x40) == 0 && f34_flash_cmd == 0)
		{
			pr_info("[touch] %s: autotest: successfully reset ic!!\n", __func__);
			break;
		}
		if(i == 500)
		{
			pr_err("[touch] %s: autotest: fail to reset ic!!\n", __func__);
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	
	
	rc = touchpad_pdtscan(g_tp);
	if (rc < 0) {
		pr_err("[touch] Failed to reads the RMI4 register map. (rc=%d)\n", rc);
		return rc;
	}
	pr_info("[touch] %s:Reflash configuration image Completed.\n", __func__);
	return rc;
}


static int SynaEraseConfigBlock(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd,f34_flash_cmd_addr,irq_status;
	uint8_t f34_flash_erase_cmd;
	int i;
	
	
	rc = SynaReadBootloadID(g_tp);
	if (rc) {
		pr_err("[touch] failed to read the Bootload ID, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	rc = SynaWriteBootloadID(g_tp);
	if (rc) {
		pr_err("[touch] failed to write the Bootload ID, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to read the F34_Flash_Control reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	
	
	f34_flash_erase_cmd = (f34_flash_cmd & 0xF0) | 0x07;
	rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_erase_cmd, 1);
	if (rc) {
		pr_err("[touch] autotest: failed to write the erase cmd of F34_flash reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_erase_cmd, rc);
		return rc;
	}
	else
	{
		pr_err("[touch] %s: autotest: send erase cmd successfully!!\n", __func__);
	}
	msleep(500); 
		
	for(i=0;;i++)
	{
		
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
		if (rc) {
			pr_err("[touch] failed to read the F01_interrupt_status reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F01_data_base+1, irq_status, rc);
			return rc;
		}
		if (irq_status & 0x1)  break;
		if(i == 500) 
		{
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	return rc;
}


static int SynaConfigBlockReflash(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr, program_enable;
	
	
	rc = SynaEraseConfigBlock(g_tp);
	if (rc) {
		pr_err("[touch] failed to erase config block (rc=%d)\n", rc);
		return rc;
	}

	
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
		pr_err("[touch] failed to read the program cmd of F34 flash reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, program_enable, rc);
		return rc;
	}
	if (program_enable != 0x80) {
		pr_err("[touch] %s: autotest:failed to erase config block!!\n", __func__);
		rc = -1;
		return rc;
	}
	else
	{
		pr_err("[touch] %s: autotest: erase config block successfully!!\n", __func__);
	}
	
	
	rc = SynaProgramConfiguration(g_tp);
	if (rc) {
		pr_err("[touch] failed to write the configuration image in config area, (rc=%d)\n",
			rc);
		return rc;
	}
	
	
	rc = SynaFinalizeReflash(g_tp);
	if (rc) {
		pr_err("[touch] failed to disable flash programming, (rc=%d)\n", rc);
		return rc;
	}
	return rc ;
}


static int touchpad_update_config(struct synaptics_tp_t *g_tp)
{
	int rc = 0;

	
	rc = SynaEnableFlashing(g_tp);
	if (rc) {
		pr_err("[touch] failed to enable flash, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}	
    
	if (g_tp->program_enable_success == 1) {
		
		rc = SynaReadConfigInfo(g_tp);
		if (rc) {
			pr_err("[touch] failed to read the config block size and block count, (rc=%d)\n",
				rc);
			return rc;
		}
		
		rc = SynaReadFirmwareInfo(g_tp);
		if (rc) {
			pr_err("[touch] failed to read the firmware block size and block count, (rc=%d)\n",
				rc);
			return rc;
		}
		
		rc = SynaConfigBlockReflash(g_tp);
		if (rc) {
			pr_err("[touch] failed to enable flash, (rc=%d)\n", rc);
			rc = -EINVAL;
			return rc;
		}
	}
	return rc;
}


static int getFileSize(char * file_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", file_name);
	pr_info("[touch], filepath=%s\n", filepath);
	pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[touch], error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL );
	return fsize;
}


#define tp_config_size		512
#define tp_config_offset	0xb100
static int readFileContent(char * file_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", file_name);
	pr_info("[touch], filepath=%s\n", filepath);
	pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("[touch], error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = tp_config_size;
	
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = tp_config_offset;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL );
	set_fs(old_fs);
	return 0;
}

int touchpad_update_config_bin(char * file_name)
{
	unsigned char* pbt_buf = 0;
	int i_ret;
	int fwsize = getFileSize(file_name);
	if (fwsize < tp_config_offset + tp_config_size) {
		pr_err("%s ERROR:Get firmware size failed\n", __func__);
		return -1;
	}
	
	pbt_buf = (unsigned char *) kmalloc(fwsize + 1, GFP_ATOMIC );
	if (readFileContent(file_name, pbt_buf)) {
		pr_err("%s() - ERROR: request_firmware failed\n", __func__);
		kfree(pbt_buf);
		return -1;
	}
	g_tp->puData = pbt_buf;
	i_ret = touchpad_update_config(g_tp);
	kfree(pbt_buf);
	if (i_ret < 0) {
		pr_err("[touch] %s: Failed to update configuration file, (rc=%d)\n", __func__, i_ret);
		return i_ret;
	}
	return i_ret;
}


static int SynaEraseFirmwareBlock(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,irq_status;
	uint8_t f34_flash_erase_cmd;
	uint8_t f34_flash_cmd;
	int i;
		
	
	rc = SynaReadBootloadID(g_tp);
	if (rc) {
		pr_err("[touch] failed to read the Bootload ID, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	rc = SynaWriteBootloadID(g_tp);
	if (rc) {
		pr_err("[touch] failed to write the Bootload ID, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to read the F34_Flash_Control reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	
	
	f34_flash_erase_cmd = (f34_flash_cmd & 0xF0) | 0x03;
	rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_erase_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to write the f34_enable_flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_erase_cmd, rc);
		return rc;
	}
	else
	{
		pr_info("[touch] %s: send erase all cmd successfully!!\n", __func__);
	}
	msleep(1000); 
		
	for(i=0;;i++)
	{
		
		rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
		if (rc) {
			pr_err("[touch] failed to read the F01_interrupt_status reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F01_data_base+1, irq_status, rc);
			return rc;
		}
		if (irq_status & 0x1)  break;
		if(i == 500) 
		{
			rc = -1;
			return rc;
		}
		msleep(10);
	}
	return rc;
}


static int SynaProgramFirmware(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t  blockNum_value[2];
	uint16_t blockNum;
	uint8_t f34_flash_cmd,f34_flash_cmd_addr;
    uint8_t irq_status;
	int i;
		
	pr_info("[touch] %s: start.\n", __func__);
	for (blockNum = 0x000; blockNum < g_tp->firmware_block_count; blockNum++)
	{
		blockNum_value[0] = blockNum & 0x00ff;
		blockNum_value[1] = (blockNum & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base, &blockNum_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write the f34_blockNum reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F34_data_base, blockNum_value[0], rc);
			return rc;
		}
		
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F34_data_base+2, g_tp->puFirmwareData, (int)g_tp->firmware_block_size);
		if (rc) {
			pr_err("[touch] failed to write the f34_block_data([0x%x]=0x%s), (rc=%d)\n",
				g_tp->pdt_map.F34_data_base+2, g_tp->puFirmwareData, rc);
			return rc;
		}
		g_tp->puFirmwareData += (int)g_tp->firmware_block_size;
		
		f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
		rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			pr_err("[touch] failed to read the F34_Flash_Control reg([0x%x]=0x%x), (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		
		
		f34_flash_cmd = (f34_flash_cmd & 0xF0) | 0x02;
		rc = touchpad_write_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
		if (rc) {
			pr_err("[touch] failed to write the f34_enable_flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
				f34_flash_cmd_addr, f34_flash_cmd, rc);
			return rc;
		}
		msleep(10); 
		
		for(i=0;;i++)
		{
			
			rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base+1, &irq_status, 1); 
			if (rc) {
				pr_err("[touch] failed to read the F01_interrupt_status reg([0x%x]=0x%x), (rc=%d)\n",
					g_tp->pdt_map.F01_data_base+1, irq_status, rc);
				return rc;
			}
			if (irq_status & 0x1)  break;
			if(i == 500) 
			{
				rc = -1;
				return rc;
			}
			msleep(10);
		}
	}	
	return rc;
}


static int SynaFirmwareBlockReflash(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,program_enable;

	
	rc = SynaEraseFirmwareBlock(g_tp);
	if (rc) {
		pr_err("[touch] failed to erase the firmware block, (rc=%d)\n", rc);
		return rc;
	}
	
	
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &program_enable, 1);
	if (rc) {
		pr_err("[touch] failed to read the program cmd of F34_Flash_Ctrl reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, program_enable, rc);
		return rc;
	}
	if (program_enable != 0x80) {
		pr_err("[touch] %s: failed to erase all!!\n", __func__);
		rc = -1;
		return rc;
	}
	else
	{
		pr_info("[touch] %s: erase all successfully!!\n", __func__);
	}
	
	
	rc = SynaProgramFirmware(g_tp);
	if (rc) {
		pr_err("[touch] failed to write the firmware image in firmware area, (rc=%d)\n",
			rc);
		return rc;
	}
	
	
	rc = SynaProgramConfiguration(g_tp);
	if (rc) {
		pr_err("[touch] failed to write the config image in config area, (rc=%d)\n",
			rc);
		return rc;
	}
	
	
	rc = SynaFinalizeReflash(g_tp);
	if (rc) {
		pr_err("[touch] %s: failed to disable flash programming. (rc=%d)\n",
			__func__, rc);
		return rc;
	}
	return rc ;
}


static int touchpad_update_firmware_img(struct synaptics_tp_t *g_tp)
{
	int rc = 0;

	
	pr_info("[touch] %s: start.\n", __func__);
	rc = SynaEnableFlashing(g_tp);
	if (rc) {
		pr_err("[touch] failed to enable flash, (rc=%d)\n", rc);
		rc = -EINVAL;
		return rc;
	}
	
	if (g_tp->program_enable_success == 1) {
		
		rc = SynaReadConfigInfo(g_tp);
		if (rc) {
			pr_err("[touch] failed to read the config block size and block count, (rc=%d)\n",
				rc);
			return rc;
		}
		
		rc = SynaReadFirmwareInfo(g_tp);
		if (rc) {
			pr_err("[touch] failed to read the firmware block size and block count, (rc=%d)\n",
				rc);
			return rc;
		}
		
		rc = SynaFirmwareBlockReflash(g_tp);
		if (rc) {
			pr_err("[touch] failed to enable flash, (rc=%d)\n", rc);
			rc = -EINVAL;
			return rc;
		}
	}
    return rc;
}


static int touchpad_read_fw_ver(void)
{
	int rc = 0;
	uint8_t fw_ver_addr,fw_value[3];
	
	fw_ver_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query18;
	rc = touchpad_read_i2c(g_tp->client, fw_ver_addr, &fw_value[0], 3);
	if (rc < 0) {
		pr_err("[touch] Failed to read the F01 query reg([0x%x]=0x%x), (rc=%d)\n", 
			fw_ver_addr, fw_value[0], rc);
		return rc;
	}
	g_tp->fw_version = fw_value[2] << 16 | fw_value[1] << 8 | fw_value[0];
	pr_info("[touch] the fw version is %d\n", (int)g_tp->fw_version);
	return 0;
}


static int touchpad_read_config_id(void)
{
	int rc = 0;
	uint8_t f34_ctrl_value[4];
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F34_control_base, &f34_ctrl_value[0], 4);
	if (rc < 0) {
		pr_err("[touch] Failed to read the config_id of F34 ctrl reg([0x%x]=0x%x), (rc=%d)\n", 
			g_tp->pdt_map.F11_control_base, f34_ctrl_value[0], rc);
		return rc;
	}
	g_tp->config_id = f34_ctrl_value[0] << 24 | f34_ctrl_value[1] << 16 | f34_ctrl_value[2] << 8 | f34_ctrl_value[3];
	pr_info("[touch] the config id in module is 0x%x\n", g_tp->config_id);
	return 0;
}

static uint16_t get_module_tp_in_bootloader_mode(void)
{
	int rc = 0;
	uint8_t pid_addr,pid_value[11];
	
	pid_value[10] = 0;
	pid_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query11;
	rc = touchpad_read_i2c(g_tp->client, pid_addr, &pid_value[0], 10);
	if (rc < 0) {
		pr_err("[touch] %s: Failed to read the product id in bootloader mode.\n", __func__);
		return INVALID_TP;
	}
	pr_info("[touch] the product id is [%s]\n", pid_value);
	if(strncmp(pid_value,"JTPB1000", strlen("JTPB1000")) == 0) return JTOUCH_BLACK_TP;
	else if(strncmp(pid_value,"JTPW1100", strlen("JTPW1100")) == 0) return JTOUCH_WHITE_TP;
	else if(strncmp(pid_value,"IF1_TR_B01", strlen("IF1_TR_B01")) == 0) return TRULY_BLACK_TP;
	else if(strncmp(pid_value,"F5_TPS_B01", strlen("F5_TPS_B01")) == 0) return TPK_SNW_BLACK_TP;
	else if(strncmp(pid_value,"F5_TPI_B01", strlen("F5_TPI_B01")) == 0) return TPK_ITO_BLACK_TP;
	else if(strncmp(pid_value,"A3i_OF_B01", strlen("A3i_OF_B01")) == 0) return T3_OFILM_BLACK_TP;
	else return INVALID_TP;
}

static void get_module_tp_and_config_id(uint16_t *module_tp,uint32_t *config_id, uint8_t is_ic_in_bootloader_mode)
{
	*module_tp = get_module_tp_in_bootloader_mode();
	if(is_ic_in_bootloader_mode)
	{
		pr_info("[touch] It's in bootloader mode!!\n");
	}
	else
	{
		
		
		*config_id = g_tp->config_id;
		pr_info("[touch] It's in UI mode!!\n");
	}
}


static int touchpad_check_module_tp(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f34_flash_cmd_addr,f34_flash_cmd;
	uint16_t module_tp;
	uint32_t config_id = 0x0;
	uint8_t is_ic_in_bootloader_mode;
	g_tp->puData = 0x0;
	g_tp->puFirmwareData = 0x0;
	
	
	touchpad_read_fw_ver();
	
	touchpad_read_config_id();
	
	f34_flash_cmd_addr = g_tp->pdt_map.F34_data_base + f34_flash_data03;
	rc = touchpad_read_i2c(g_tp->client, f34_flash_cmd_addr, &f34_flash_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to read the F34_Flash_cmd reg([0x%x]=0x%x), (rc=%d)\n",
			f34_flash_cmd_addr, f34_flash_cmd, rc);
		return rc;
	}
	is_ic_in_bootloader_mode = f34_flash_cmd & 0x80;
	get_module_tp_and_config_id(&module_tp, &config_id, is_ic_in_bootloader_mode);
	pr_info("[touch] TP module config id: 0x%x.\n", config_id);
	switch(module_tp) {
	case JTOUCH_BLACK_TP:
		pr_info("[touch] TP module is J-Touch and the touch panel is black.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_6_0_33_FW_VER || IS_UNDER_TESTING) 
		{
			pr_info("[touch] It'll update the latest FW Ver. to %d\n", SYNAP_DS4_3_6_0_33_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1296077_img;
			g_tp->puData = S3202_Config_JTP_black_31303032_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				pr_err("[touch] %s: Failed to update F/W, (rc=%d)\n", __func__, rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_02_JTP_CONFIG_ID) { 
				pr_info("[touch] It'll update the latest CONFIG ID to 0x%x\n", SYNAP_02_JTP_CONFIG_ID);
				g_tp->puData = S3202_Config_JTP_black_31303032_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					pr_err("[touch] %s: Failed to update configuration file, (rc=%d)\n", __func__, rc);
					return rc;
				}
			}
		}
		break;
	case JTOUCH_WHITE_TP:
		pr_info("[touch] It's the J-Touch white tp module.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_6_0_33_FW_VER || IS_UNDER_TESTING) 
		{
			pr_info("[touch] It'll update the latest FW Ver. to %d\n", SYNAP_DS4_3_6_0_33_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1296077_img;
			g_tp->puData = S3202_Config_JTP_white_31313032_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				pr_err("[touch] %s: Failed to update F/W, (rc=%d)\n", __func__, rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_02_JTP_CONFIG_ID) { 
				pr_info("[touch] It'll update the latest CONFIG ID to 0x%x\n", SYNAP_02_JTP_CONFIG_ID);
				g_tp->puData = S3202_Config_JTP_white_31313032_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					pr_err("[touch] %s: Failed to update configuration file, (rc=%d)\n", __func__, rc);
					return rc;
				}
			}
		}
		break;
	case TRULY_BLACK_TP:
		pr_info("[touch] It's the Truly black tp module.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_6_0_34_FW_VER || IS_UNDER_TESTING) 
		{
			pr_info("[touch] It'll update the latest FW Ver. to %d\n", SYNAP_DS4_3_6_0_33_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1365481_img;
			g_tp->puData = S3202_Config_TRL_black_52423230_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				pr_err("[touch] %s: Failed to update F/W, (rc=%d)\n", __func__, rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_02_TRL_CONFIG_ID) { 
				pr_info("[touch] It'll update the latest CONFIG ID to 0x%x\n", SYNAP_02_JTP_CONFIG_ID);
				g_tp->puData = S3202_Config_TRL_black_52423230_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					pr_err("[touch] %s: Failed to update configuration file, (rc=%d)\n", __func__, rc);
					return rc;
				}
			}
		}
		break;
	case TPK_SNW_BLACK_TP:
		pr_info("[touch] It's the TPK SNW black tp.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_6_0_35_FW_VER || IS_UNDER_TESTING) 
		{
			pr_info("[touch] It'll update the latest FW Ver. to %d\n", SYNAP_DS4_3_6_0_33_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1471960_img;
			g_tp->puData = S3202_Config_TPKSNW_black_53423130_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				pr_err("[touch] %s: Failed to update F/W, (rc=%d)\n", __func__, rc);
				return rc;
			}
		}
		else
		{
			if (config_id != SYNAP_10_TPKSNW_CONFIG_ID) { 
				pr_info("[touch] It'll update the latest CONFIG ID to 0x%x\n", SYNAP_02_JTP_CONFIG_ID);
				g_tp->puData = S3202_Config_TPKSNW_black_53423130_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					pr_err("[touch] %s: Failed to update configuration file, (rc=%d)\n", __func__, rc);
					return rc;
				}
			}
		}
		break;
	case T3_OFILM_BLACK_TP:
		pr_info("[touch] It's the T3 O-Film black tp.\n");
		if(is_ic_in_bootloader_mode != 0 || g_tp->fw_version != SYNAP_DS4_3_2_2_FW_VER || IS_UNDER_TESTING) 
		{
			pr_info("[touch] It'll update the latest FW Ver. to %d\n", SYNAP_DS4_3_2_2_FW_VER);
			g_tp->puFirmwareData = S3202_Firmware_1200567_img;
			g_tp->puData = s3202_Config_OFILM_black_4F423134_img;
			rc = touchpad_update_firmware_img(g_tp);
			if (rc < 0) {
				pr_err("[touch] %s: Failed to update F/W, (rc=%d)\n", __func__, rc);
				return rc;
			}
		}
		else
		{
			if (config_id < SYNAP_T3_OFILM_CONFIG_ID_04) { 
				pr_info("[touch] It'll update the latest CONFIG ID to 0x%x\n", SYNAP_T3_OFILM_CONFIG_ID_04);
				g_tp->puData = s3202_Config_OFILM_black_4F423134_img;
				rc = touchpad_update_config(g_tp);
				if (rc < 0) {
					pr_err("[touch] %s: Failed to update configuration file, (rc=%d)\n", __func__, rc);
					return rc;
				}
			}
		}
		break;
	case TPK_ITO_BLACK_TP:
		pr_info("[touch] It's the TPK ITO black tp.\n");
		break;
	default:
		pr_err("[touch] %s: unknown tp module!!, module id: %d\n", __func__, module_tp);
	} 
	return rc;
}


static int touchpad_get_raw_cap_pass_sepc(void)
{
	int rc = 0;
	uint16_t module_tp;
	
	module_tp = get_module_tp_in_bootloader_mode();
	pr_info("[touch] It's in UI mode.\n");

	
	g_tp->isKeyAaCriteraDiff=0;

	switch(module_tp) {
	case JTOUCH_BLACK_TP:	
		
		g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_JT_BLACK_TP;
		g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_JT_BLACK_TP;
		break;
	case JTOUCH_WHITE_TP:
		
		g_tp->f54_raw_cap_min_limit = F54_RAW_CAP_MIN_LIMIT_JT_WHITE_TP;
		g_tp->f54_raw_cap_max_limit = F54_RAW_CAP_MAX_LIMIT_JT_WHITE_TP;
		break;
	case TRULY_BLACK_TP:	
		
		g_tp->f54_raw_cap_min_limit = F54_AA_AREA_RAW_CAP_MIN_LIMIT_TR_BLACK_TP;
		g_tp->f54_raw_cap_max_limit = F54_AA_AREA_RAW_CAP_MAX_LIMIT_TR_BLACK_TP;
		g_tp->f54_key_raw_cap_min_limit = F54_KEY_RAW_CAP_MIN_LIMIT_TR_BLACK_TP;
		g_tp->f54_key_raw_cap_max_limit = F54_KEY_RAW_CAP_MAX_LIMIT_TR_BLACK_TP;
		pr_info("[key] the raw_cap pass criteria = (%d ~ %d).\n", g_tp->f54_key_raw_cap_min_limit, g_tp->f54_key_raw_cap_max_limit);
		break;
	case TPK_SNW_BLACK_TP:	
		
		g_tp->f54_raw_cap_min_limit = F54_AA_AREA_RAW_CAP_MIN_LIMIT_TR_BLACK_TP;
		g_tp->f54_raw_cap_max_limit = F54_AA_AREA_RAW_CAP_MAX_LIMIT_TR_BLACK_TP;
		break;
	case T3_OFILM_BLACK_TP:
		
		g_tp->f54_raw_cap_min_limit = F54_AA_AREA_RAW_CAP_MIN_LIMIT_T3_OFILM_BLACK_TP;
		g_tp->f54_raw_cap_max_limit = F54_AA_AREA_RAW_CAP_MAX_LIMIT_T3_OFILM_BLACK_TP;
		g_tp->f54_key_raw_cap_min_limit = F54_KEY_RAW_CAP_MIN_LIMIT_OF_T3_OFILM_BLACK_TP;
		g_tp->f54_key_raw_cap_max_limit = F54_KEY_RAW_CAP_MAX_LIMIT_OF_T3_OFILM_BLACK_TP;
		g_tp->isKeyAaCriteraDiff=1;
		
		g_tp->capKeyTxLinValid[0]=2;
		g_tp->capKeyTxLinValid[1]=5;
		g_tp->capKeyTxLinValid[2]=9;
		break;
	default:
		pr_err("[touch] %s: unknown tp module, module id: %d\n", __func__, module_tp);
	} 
	pr_err("[touch] the raw_cap pass criteria = (%d ~ %d).\n", g_tp->f54_raw_cap_min_limit, g_tp->f54_raw_cap_max_limit);
	return rc;
}


static int touchpad_detect_synaptics(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	uint8_t f01_value[2];
	uint8_t page_select = 0x0;
	uint8_t f11_addr,f11_value[2];
	
	
	rc = touchpad_pdtscan(g_tp);
	if (rc < 0) {
		pr_err("[touch] Failed to scan the RMI4 register map. (rc=%d)\n", rc);
		return rc;
	}
	
	rc = touchpad_write_i2c(g_tp->client, PAGE_SELECT_REGISTER, &page_select, sizeof(page_select)); 
	if (rc) {
		pr_err("[touch] Failed to write the RMI4 page table(addr[0x%x]:0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
	
	
	rc = touchpad_check_module_tp(g_tp);
	if (rc) {
		pr_err("[touch] Failed to check module tp. (rc=%d)\n", rc);
		return rc;
	} else {
		
		touchpad_read_fw_ver();
		
		touchpad_read_config_id();
		
		touchpad_get_raw_cap_pass_sepc();
	}
	
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_data_base, &f01_value[0], 2);
	if (rc < 0) {
		pr_err("[touch] Failed to read the irq status(addr[0x%x]:0x%x), (rc=%d)\n", 
			g_tp->pdt_map.F01_data_base, f01_value[0], rc);
		return rc;
	}
	
	
	if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
		(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER) {
		f11_addr = (g_tp->pdt_map.F11_control_base + DS4_3_2_F11_2D_CTRL77);
		rc = touchpad_read_i2c(g_tp->client, f11_addr, &f11_value[0], 2);
		if (rc < 0) {
			pr_err("[touch] Failed to read the F11 data(addr[0x%x]:0x%x), (rc=%d)\n", 
				f11_addr, f11_value[0], rc);
			return rc;
		}
		g_tp->number_of_rx_electrodes = (int)f11_value[0];
		g_tp->number_of_tx_electrodes = (int)f11_value[1];
		pr_info("[touch] the number of the TX(%d) * RX(%d) channels\n", 
			g_tp->number_of_tx_electrodes, g_tp->number_of_rx_electrodes);
	} else if ((int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER) {
		f11_addr = (g_tp->pdt_map.F11_control_base + DS4_3_6_F11_2D_CTRL77);
		rc = touchpad_read_i2c(g_tp->client, f11_addr, &f11_value[0], 2);
		if (rc < 0) {
			pr_err("[touch] Failed to read F11 data(addr[0x%x]:0x%x), (rc=%d)\n", 
				f11_addr, f11_value[0], rc);
			return rc;
		}
		g_tp->number_of_rx_electrodes = (int)f11_value[0];
		g_tp->number_of_tx_electrodes = (int)f11_value[1];
		pr_info("[touch] the number of the TX(%d) * RX(%d) channels\n", 
			g_tp->number_of_tx_electrodes, g_tp->number_of_rx_electrodes);
	} 	else if ((int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER) {
		f11_addr = (g_tp->pdt_map.F11_control_base + DS4_3_6_1_F11_2D_CTRL77);
		rc = touchpad_read_i2c(g_tp->client, f11_addr, &f11_value[0], 2);
		if (rc < 0) {
			pr_err("[touch] Failed to read F11 data(addr[0x%x]:0x%x), (rc=%d)\n", 
				f11_addr, f11_value[0], rc);
			return rc;
		}
		g_tp->number_of_rx_electrodes = (int)f11_value[0];
		g_tp->number_of_tx_electrodes = (int)f11_value[1];
		pr_info("[touch] the number of the TX(%d) * RX(%d) channels\n", 
			g_tp->number_of_tx_electrodes, g_tp->number_of_rx_electrodes);
	} else {
		pr_err("[touch] the TX * RX channels are not in range!\n");
	}

	g_tp->number_of_rx_electrodes ++;
	return 0;
}


static int touchpad_config_gpio(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	
    gpio_set_value(g_tp->gpio_rst, 1);
	pr_debug("[touch] set hw reset to be high, then delay 30ms!!\n");
	msleep(30);
    gpio_set_value(g_tp->gpio_rst, 0);
    pr_debug("[touch] set hw reset to be low, then delay 1ms!!\n");
    msleep(1);
	
    gpio_set_value(g_tp->gpio_rst, 1);
    pr_debug("[touch] set hw reset to be high, then delay 1ms!!\n");
	msleep(1); 
    return rc;
}


static int touchpad_release_gpio(struct synaptics_tp_t *g_tp)
{
    int rc = 0;

	gpio_free(g_tp->gpio_irq);
	gpio_free(g_tp->gpio_rst);
	return rc;
}

static int touchpad_setup_gpio(struct synaptics_tp_t *g_tp)
{
    int rc = 0;

	
    rc = gpio_request(g_tp->gpio_irq, "synaptics_ts_irq");
    if (rc) {
    	pr_err("[touch] Failed to request irq_gpio(= %d), (rc=%d)\n", g_tp->gpio_irq, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_input(g_tp->gpio_irq);
    if (rc) {
        pr_err("[touch]failed to configure irq_gpio(%d) direction, (rc=%d)\n", g_tp->gpio_irq, rc);
		goto err_gpio_config;
    }
	
			
	rc = gpio_request(g_tp->gpio_rst, "synaptics_ts_rst");
	if (rc)
    {
        pr_err("[touch] Failed to request rst_gpio(%d), (rc=%d)\n", g_tp->gpio_rst, rc);
		goto err_gpio_config;
    }

    rc = gpio_direction_output(g_tp->gpio_rst, 0);
    if ( rc )
    {
        pr_err("[touch] Failed to configure rst_gpio(%d), (rc=%d)\n", g_tp->gpio_rst, rc);
        goto err_gpio_config;
    }
	
	return rc;
	
err_gpio_config:
	touchpad_release_gpio(g_tp);
	return rc;
}

static int touchpad_power_on_device(struct synaptics_tp_t *g_tp, int on)
{
	int rc = 0;
	static int prev_on = 0;

	if (on == prev_on) {
		return 0;
	}

	if(on) {
		
		g_tp->ldo19_regulator = regulator_get(&g_tp->client->dev, "vdd");
		if (IS_ERR(g_tp->ldo19_regulator)) {
			rc = PTR_ERR(g_tp->ldo19_regulator);
			pr_err("[touch] %s: failed to get ldo19 regulator, (rc=%d)\n",
							__func__, rc);
			rc = -ENODEV;
			goto exit;
		}
		
		
		g_tp->lvs1_regulator = regulator_get(&g_tp->client->dev, "vcc_i2c");
		if (IS_ERR(g_tp->lvs1_regulator)) {
			rc = PTR_ERR(g_tp->lvs1_regulator);
			pr_err("[touch] %s: failed to get lvs1 regulator, (rc=%d)\n",
							__func__, rc);
			rc = -ENODEV;
			goto get_lvs1_fail;
		}
		
		
		
		rc = regulator_set_voltage(g_tp->ldo19_regulator, 2850000, 2850000);
		if (rc) {
			pr_err("[touch] failed to set ldo19 regulator, (rc=%d)\n", rc);
			goto set_ldo19_fail;
		}
	    
		
		rc = regulator_enable(g_tp->ldo19_regulator);
		if (rc) {
			pr_err("[touch] failed to enable ldo19 regulaotr, (rc=%d)\n", rc);
			goto set_ldo19_fail;
		}
		msleep(5);
		
		
		rc = regulator_enable(g_tp->lvs1_regulator);
		if (rc) {
			pr_err("[touch] failed to enable lvs1 regulator, (rc=%d)\n", rc);
			goto set_lvs1_fail;
		}
		msleep(5);
		
		pr_info("[touch] power on device successfully.\n");			
#ifdef CONFIG_PM_LOG
		PM_LOG_EVENT(PM_LOG_ON, PM_LOG_TOUCH);
#endif	
		prev_on = on;
		return 0;	
	} else {
		pr_info("Power off device.\n");
#ifdef CONFIG_PM_LOG
		PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_TOUCH);
#endif 
		prev_on = on;
	}


	regulator_disable(g_tp->lvs1_regulator);
set_lvs1_fail:
	regulator_disable(g_tp->ldo19_regulator);
set_ldo19_fail:
	regulator_put(g_tp->lvs1_regulator);
get_lvs1_fail:
	regulator_put(g_tp->ldo19_regulator);
exit:
	return rc;
}

#if 0

static void touchpad_create_kernel_debuglevel(void)
{
	SYNAPTICS_PRINTK(1, "create kernel debuglevel!!!\n");
	SYNAPTICS_PRINTK(1, "create kernel debuglevel!!!\n");
	if (kernel_debuglevel_dir!=NULL) {
		debugfs_create_u32("ts_flow_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&SYNAPTICS_DLL));
		debugfs_create_u32("ts_event_dll", S_IRUGO | S_IWUGO,
			kernel_debuglevel_dir, (u32 *)(&SYNAPTICS_TOUCH_REPORT_DLL));
		debugfs_create_u32("key_event_dll", S_IRUGO | S_IWUGO,
				kernel_debuglevel_dir, (u32 *)(&SYNAPTICS_CAPKEY_REPORT_DLL));
			
	} else {
		printk(KERN_ERR "failed to create SYNAPTICS mXT224E touch dll in debuglevel dir!!!\n");
	}
}

static void touchpad_destroy_kernel_debuglevel(void)
{
	SYNAPTICS_PRINTK(1, "destroy kernel debuglevel!!!\n");

	if (kernel_debuglevel_dir)
		debugfs_remove_recursive(kernel_debuglevel_dir);
}
#endif


static int touchpad_F54_set_report_size(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	int rx = g_tp->number_of_rx_electrodes;
    int tx = g_tp->number_of_tx_electrodes;
	
	switch (g_tp->report_type) {
	case F54_HIGH_RESISTANCE:
		g_tp->report_size = F54_HIGH_RESISTANCE_READ_BYTES / 2;
		break;
	case F54_TX_TX_SHORT:
		g_tp->report_size = F54_TX_TX_SHORT_READ_BYTES;
		break;
	case F54_RAW_CAPACITANCE:
		g_tp->report_size = 2 * rx * tx;
		break;
	case F54_RX_RX_SHORT_7:
		g_tp->report_size = 2 * rx * tx;
		break;
	case F54_RX_RX_REPORT_17:
		g_tp->report_size = 2 * rx * (rx - tx);
		break;		
	default:
		g_tp->report_size = 0;
	}
	DBG_REG("the F54 report type = %d ,test size:%d\n",
			(int)g_tp->report_type, g_tp->report_size);	
	return rc;
}


static int touchpad_F54_report_type_17(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j,k=0,m=0;
	uint16_t command;
	uint8_t command_value[2];

	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write low and high index data[0x%x]=0x%x, (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &g_tp->report_type17[m],1);
		if (rc) {
			pr_err("[touch] failed to read report data([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, g_tp->report_type17[m], rc);
			return rc;
		}
		DBG_REG("the report_type17[%d]=0x%x\n", m,g_tp->report_type17[m]);
		m=m+1;
	}
	
	
	for (i = 0; i < (g_tp->number_of_rx_electrodes - g_tp->number_of_tx_electrodes); i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			g_tp->rx_rx_imagearray_17[i][j] = (g_tp->report_type17[k] | (g_tp->report_type17[k+1] << 8));
			DBG_REG("data17[%d][%d]= %4d", i, j, (int)g_tp->rx_rx_imagearray_17[i][j]);
			k = k + 2;
	   }
	   
	}	
	return rc;
}


static int touchpad_F54_report_type_7(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j,k=0,m=0;
	uint16_t command;
	uint8_t command_value[2];

	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write low and high index data[0x%x]=0x%x, (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &g_tp->report_type7[m],1);
		if (rc) {
			pr_err("[touch] failed to read report data[0x%x]=0x%x, (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, g_tp->report_type7[m], rc);
			return rc;
		}
		m=m+1;
	}

	
	for (i = 0; i < g_tp->number_of_tx_electrodes; i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			g_tp->rx_rx_imagearray_7[i][j] = (g_tp->report_type7[k] | (g_tp->report_type7[k+1] << 8));
			DBG_REG("data7[%d][%d]= %4d", i, j, (int)g_tp->rx_rx_imagearray_7[i][j]);
			k = k + 2;			
	   }
	   
	}
	return rc;
}


static int touchpad_F54_raw_capacitance(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j=0,k=0;
	uint16_t command;
	uint8_t command_value[2];
	int count = 0;
	
	pr_info("[touch] F54_raw_cap_pass_spec=%d ~ %d\n", g_tp->f54_raw_cap_min_limit, g_tp->f54_raw_cap_max_limit);
	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &g_tp->raw_cap_value[j],1);
		if (rc) {
			pr_err("[touch] failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, g_tp->raw_cap_value[j], rc);
			return rc;
		}
		DBG_REG("the raw_cap_value[%d]=0x%x\n", j,g_tp->raw_cap_value[j]);
		j=j+1;
	}
	
	
	for (i = 0; i < g_tp->number_of_tx_electrodes; i++) {
		pr_info("[touch] [%d]: ",i);
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			g_tp->raw_cap[i][j] = (g_tp->raw_cap_value[k] | (g_tp->raw_cap_value[k+1] << 8));
			k = k + 2;
			pr_info("  %4d  ", (int)g_tp->raw_cap[i][j]);
		}
		pr_info("\n");
	}
	
	
	for (i = 0; i < g_tp->number_of_tx_electrodes; i++) {
		for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			if (1==g_tp->isKeyAaCriteraDiff)
			{
				if (j == g_tp->number_of_rx_electrodes - 1) {
					
					pr_info("capKeyTxLinValid[0]=%d capKeyTxLinValid[1]=%d capKeyTxLinValid[2]=%d\n",g_tp->capKeyTxLinValid[0],g_tp->capKeyTxLinValid[1],g_tp->capKeyTxLinValid[2]);
					if (i != g_tp->capKeyTxLinValid[0] && i != g_tp->capKeyTxLinValid[1] && i != g_tp->capKeyTxLinValid[2]) {
						pr_info("[touch], raw[%d][%d] not touch area, skip test!", i, j);
					}
					else
					{
						if (g_tp->raw_cap[i][j] >= g_tp->f54_key_raw_cap_min_limit &&
							g_tp->raw_cap[i][j] <= g_tp->f54_key_raw_cap_max_limit) {
							count += 0;
							pr_info("key raw_cap_value[%d][%d]= %d Pass min=%d max=%d\n", i,j,(int)g_tp->raw_cap[i][j],g_tp->f54_key_raw_cap_min_limit, g_tp->f54_key_raw_cap_max_limit);
						} else {
							count += 1;
							pr_info("[touch] key raw[%d][%d]= %d (fail) min=%d max=%d\n", i,j,(int)g_tp->raw_cap[i][j],g_tp->f54_key_raw_cap_min_limit, g_tp->f54_key_raw_cap_max_limit);
						}
					}
				}
				else
				{
					
					if (g_tp->raw_cap[i][j] >= g_tp->f54_raw_cap_min_limit &&
					g_tp->raw_cap[i][j] <= g_tp->f54_raw_cap_max_limit) {
						count += 0;
							pr_info("AA raw_cap_value[%d][%d]= %d Pass min=%d max=%d\n", i,j,(int)g_tp->raw_cap[i][j],g_tp->f54_raw_cap_min_limit, g_tp->f54_raw_cap_max_limit);
					} else {
						count += 1;
						pr_info("[touch] AA raw[%d][%d]= %d (fail)\n", i,j,(int)g_tp->raw_cap[i][j]);
					}
				}
			}
			else
			{
				
				if (g_tp->raw_cap[i][j] >= g_tp->f54_raw_cap_min_limit &&
					g_tp->raw_cap[i][j] <= g_tp->f54_raw_cap_max_limit) {
					count += 0;
					pr_info("AA raw_cap_value[%d][%d]= %d Pass min=%d max=%d\n", i,j,(int)g_tp->raw_cap[i][j],g_tp->f54_raw_cap_min_limit, g_tp->f54_raw_cap_max_limit);
				} else {
					count += 1;
					pr_info("[touch] AA raw[%d][%d]= %d (fail)\n", i,j,(int)g_tp->raw_cap[i][j]);
				}
			}
#if 0
			} else {
				
				if (j < (g_tp->number_of_rx_electrodes)) {
					
					if (g_tp->raw_cap[i][j] >= g_tp->f54_raw_cap_min_limit &&
						g_tp->raw_cap[i][j] <= g_tp->f54_raw_cap_max_limit) {
						count += 0;
						DBG_REG("raw_cap_value[%d][%d]= %d Pass\n", i,j,(int)g_tp->raw_cap[i][j]);
					} else {
						count += 1;
						pr_info("[touch] raw[%d][%d]= %d (fail)\n", i,j,(int)g_tp->raw_cap[i][j]);
					}
				} else {
					
				}
			}
#endif
		}
	}

	if (count == 0) {
		g_tp->F54_testing_flag += 0;
		pr_info("[touch] raw capacitance testing Pass.\n");
	} else {
		g_tp->F54_testing_flag += RAW_CAPACITANCE;
		pr_info("[touch] raw capacitance testing Fail.\n");
	}
	return rc;
}


static int touchpad_F54_TX_TX_short(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i = 0;
	int count = 0;
	uint16_t command;
	uint8_t command_value[2],tx_tx_value[g_tp->report_size];

	for (command = 0x0000; command < g_tp->report_size; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write low and high index data[0x%x]=0x%x, (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &tx_tx_value[i],1);
		if (rc) {
			pr_err("[touch] failed to read report data[0x%x]=0x%x, (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, tx_tx_value[i], rc);
			return rc;
		}
		i=i+1;
	}
	
	
	for (i = 0; i < g_tp->report_size; i++) {
		if (tx_tx_value[i] == 0x0) {
			count += 0;
			pr_info("[touch] Tx-Tx value[%d]=%d\n",i,tx_tx_value[i]);
		} else {
			count += 1;
			pr_info("[touch] Tx-Tx[%d]=%d (fail).\n",i,tx_tx_value[i]);
		}
	}
	
	if (count == 0) {
		g_tp->F54_testing_flag += 0;
		pr_info("[touch] Tx-Tx short testing Pass.\n");
	} else {
		g_tp->F54_testing_flag += TX_TX_SHORT_ERR;
		pr_info("[touch] Tx-Tx short testing Fail.\n");
	}
	return rc;
}


static int touchpad_F54_hgih_resistance(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i,j=0,k=0;
	uint16_t command,high_resistance[g_tp->report_size];
	uint8_t command_value[2],high_resistance_value[F54_HIGH_RESISTANCE_READ_BYTES];

	for (command = 0x0000; command < F54_HIGH_RESISTANCE_READ_BYTES; command++)
	{
		command_value[0] = command & 0x00ff;
		command_value[1] = (command & 0xff00) >> 8;
		
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base+1, &command_value[0], 2);
		if (rc) {
			pr_err("[touch] failed to write low and high index data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+1, command_value[0], rc);
			return rc;
		}
		
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_data_base+3, &high_resistance_value[j],1);
		if (rc) {
			pr_err("[touch] failed to read report data[0x%x]:0x%x (rc=%d)\n",
				g_tp->pdt_map.F54_data_base+3, high_resistance_value[j], rc);
			return rc;
		}
		DBG_REG("the high resistance value[%d]=0x%x\n", j,high_resistance_value[j]);
		j=j+1;
	}
	
	
	for (i = 0; i < g_tp->report_size; i++) 
	{
		high_resistance[i] = (high_resistance_value[k] | (high_resistance_value[k+1] << 8));
		pr_info("[touch] the high resistance value[%d]=%d\n", i,(int)high_resistance[i]);
		k = k + 2;
	}
	g_tp->F54_testing_flag += 0;
#if 0
	for (i = 0; i < g_tp->report_size; i++) {
		high_resistance[i] = (high_resistance_value[k] | (high_resistance_value[k+1] << 8));
		if (i <= 1) {
			if ((int)high_resistance[i] > F54_HIGH_RESISTANCE_RX_LIMIT) {
				g_tp->F54_testing_flag = HIGH_RESISTANCE_ERR;
				pr_info("[touch] the high_resistance value[%d]=%d (fail)\n", i,(int)high_resistance[i]);
			}else {
				g_tp->F54_testing_flag += 0;
				pr_info("[touch] the high_resistance value[%d]=%d\n", i,(int)high_resistance[i]);
			} 
		} else {
			if (high_resistance[i] < F54_HIGH_RESISTANCE_MIN_LIMIT) {
				g_tp->F54_testing_flag = HIGH_RESISTANCE_ERR;
				pr_info("[touch] the high_resistance value[%d]=%d (fail)\n", i,(int)high_resistance[i]);
			} else {
				g_tp->F54_testing_flag += 0;
				pr_info("[touch] the high_resistance value[%d]=%d\n", i,(int)high_resistance[i]);
			}
        }
		k = k + 2;
	}

	
	if (g_tp->F54_testing_flag == 0)
	{
		pr_info("[touch] the high resistance testing Pass.\n");
	}	
	else
	{
		pr_info("[touch] the high resistance testing Fail.\n");
	}
#endif	
	return rc;
}


static int touchpad_F54_testing(struct synaptics_tp_t *g_tp)
{
	struct i2c_client *client = g_tp->client;
	int rc = 0;
    uint8_t page_select,get_report,irq_cmd,interrupt_enable;
	uint8_t force_k,force_update,noise_mitigation_ctrl_addr=0x0,noise_mitigation_ctrl;
	uint8_t cbc_2d_val = 0x0,cbc_0d_val = 0x0,cbc_0d_addr = 0x0,cbc_2d_addr = 0x0,key_cbc_val = 0x0,touch_cbc_val = 0x0;
	
	
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
	
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_control_base+1, &interrupt_enable, 1);
	if (rc) {
		pr_err("[touch] failed to read the interrupt enable reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, interrupt_enable, rc);
		return rc;
	}
	
	irq_cmd = 0x00;
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base+1, &irq_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to write the interrupt enable reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, irq_cmd, rc);
		return rc;
	}
	
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_control_base+1, &irq_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to read the interrupt enable reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, irq_cmd, rc);
		return rc;
	}
	
	page_select = 0x01;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
    rc = touchpad_read_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	
	
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_data_base, &g_tp->report_type, 1);
	if (rc) {
		pr_err("[touch] failed to write the report type[0x%x]=%d, (rc=%d)\n",
			g_tp->pdt_map.F54_data_base, (int)g_tp->report_type, rc);
		return rc;
	}
	msleep(1);
	
	
	rc = touchpad_F54_set_report_size(g_tp);
	if (rc) {
		pr_err("[touch] failed to set the report size, (rc=%d)\n", rc);
		return rc;
	}
	msleep(1);
	
	if (g_tp->report_type == F54_RX_RX_SHORT_7 || g_tp->report_type == F54_RX_RX_REPORT_17)
	{
		
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
			cbc_2d_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL07;
		rc = touchpad_read_i2c(client, cbc_2d_addr, &touch_cbc_val, 1);
		if (rc) {
			pr_err("[touch] failed to read the 2D_CBC_val[0x%x]=0x%x, (rc=%d)\n",
				cbc_2d_addr, touch_cbc_val, rc);
			return rc;
		}
		cbc_2d_val = 0x0;
		rc = touchpad_write_i2c(client, cbc_2d_addr, &cbc_2d_val, 1);
		if (rc) {
			pr_err("[touch] failed to write the 2D_CBC_val[0x%x]=0x%x, (rc=%d)\n",
				cbc_2d_addr, cbc_2d_val, rc);
			return rc;
		}
		
		
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
			cbc_0d_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL57;
		rc = touchpad_read_i2c(client, cbc_0d_addr, &key_cbc_val, 1);
		if (rc) {
			pr_err("[touch] failed to read the 0D_CBC[0x%x]=0x%x, (rc=%d)\n",
				cbc_0d_addr, key_cbc_val, rc);
			return rc;
		}
		cbc_0d_val = 0x0;
		rc = touchpad_write_i2c(client, cbc_0d_addr, &cbc_0d_val, 1);
		if (rc) {
			pr_err("[touch] failed to write 0D_CBC[0x%x]=0x%x, (rc=%d)\n",
				cbc_0d_addr, cbc_0d_val, rc);
			return rc;
		}
		
		
		#if 0
		if (system_rev <= EVT1_3 && (msm8960_project_id == DETROIT)) {
			if ((int)g_tp->fw_version == SYNAP_DS4_3_0_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + f54_analog_ctrl41;
			else
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		} else {
			if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		}
		#endif
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER ||
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_35_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		noise_mitigation_ctrl = 0x1;
		rc = touchpad_write_i2c(client, noise_mitigation_ctrl_addr, &noise_mitigation_ctrl, 1);
		if (rc) {
			pr_err("[touch] failed to disable noise mitigation ctrl[0x%x]=0x%x, (rc=%d)\n",
				noise_mitigation_ctrl_addr, noise_mitigation_ctrl, rc);
			return rc;
		}
		DBG_REG("disable noise mitigation ctrl[0x%x]=0x%x\n", noise_mitigation_ctrl_addr, noise_mitigation_ctrl);
		msleep(1);
		
		
		force_update = 0x04;
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_command_base, &force_update, 1);
		if (rc) {
			pr_err("[touch] failed to force update value[0x%x]=0x%x, (rc=%d)\n",
				g_tp->pdt_map.F54_command_base, force_update, rc);
			return rc;
		}
		DBG_REG("force update value[0x%x]=0x%x\n", g_tp->pdt_map.F54_command_base, force_update);
		
		
		do {
			rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_command_base, &force_update, 1);
			if (rc) {
				pr_err("[touch] failed to read the F54 cmd base reg([0x%x]=0x%x), (rc=%d)\n",
					g_tp->pdt_map.F54_command_base, force_update, rc);
				return rc;
			}
			DBG_REG("force_update value[0x%x]=0x%x\n", g_tp->pdt_map.F54_command_base, force_update);
		} while (force_update != 0x0);
		
		
		force_k = 0x02;
		rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_command_base, &force_k, 1);
		if (rc) {
			pr_err("[touch] failed to force calibration value[0x%x]=0x%x, (rc=%d)\n",
			g_tp->pdt_map.F54_command_base, force_k, rc);
			DBG_REG("force calibration value[0x%x]=0x%x\n", g_tp->pdt_map.F54_command_base, force_k);
			return rc;
		}
		
		do {
			rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_command_base, &force_k, 1);
			if (rc) {
				pr_err("[touch] failed to read the F54 cmd base reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F54_command_base, force_k, rc);
				return rc;
			}
			DBG_REG("the F54 cmd base reg[0x%x]=0x%x\n", g_tp->pdt_map.F54_command_base, force_k);
		} while (force_k != 0x0);
	}
	
	
	get_report = 0x01;
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F54_command_base, &get_report, 1);
	if (rc) {
		pr_err("[touch] failed to get the report value[0x%x]=0x%x, (rc=%d)\n",
			g_tp->pdt_map.F54_command_base, get_report, rc);
		return rc;
	}
	msleep(1);
	do {
		
		rc = touchpad_read_i2c(client, g_tp->pdt_map.F54_command_base, &get_report, 1);
		if (rc) {
			pr_err("[touch] failed to get the report value[0x%x]=0x%x, (rc=%d)\n",
			g_tp->pdt_map.F54_command_base, get_report, rc);
			return rc;
		}
		pr_info("get the report value[0x%x]=0x%x\n", g_tp->pdt_map.F54_command_base, get_report);
	} while(get_report!=0x0);	
    
	if (get_report == 0x0) {
		
		switch (g_tp->report_type) {
		case F54_RAW_CAPACITANCE:
			rc = touchpad_F54_raw_capacitance(g_tp);
			if (rc) {
				pr_err("[touch] failed to test F54 raw_cap, the report type=%d, (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;	
		case F54_HIGH_RESISTANCE:
			rc = touchpad_F54_hgih_resistance(g_tp);
			if (rc) {
				pr_err("[touch] failed to test F54_hgih_resistance, the report type=%d, (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		case F54_TX_TX_SHORT:
			rc = touchpad_F54_TX_TX_short(g_tp);
			if (rc) {
				pr_err("[touch] failed to test F54_TX_TX_short, the report type=%d, (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		case F54_RX_RX_SHORT_7:
			rc = touchpad_F54_report_type_7(g_tp);
			if (rc) {
				pr_err("[touch] failed to test F54_RX_RX_short_7, the report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		case F54_RX_RX_REPORT_17:
			rc = touchpad_F54_report_type_17(g_tp);
			if (rc) {
				pr_err("[touch] failed to test F54_RX_RX_short_17, the report type=%d (rc=%d)\n",
					g_tp->report_type, rc);
				return rc;
			}
			mdelay(1);
			break;
		default:
			pr_err("[touch] %s: failed to test F54, the report type=%d\n",
				__func__, g_tp->report_type);
		} 
	}
	
	
	if (g_tp->report_type == F54_RX_RX_SHORT_7 || g_tp->report_type == F54_RX_RX_REPORT_17)
	{
		
		rc = touchpad_write_i2c(client, cbc_2d_addr, &touch_cbc_val, 1);
		if (rc) {
			pr_err("[touch] failed to write the 2D_CBC_val[0x%x]=0x%x, (rc=%d)\n",
				cbc_2d_addr, touch_cbc_val, rc);
			return rc;
		}
		
		
		rc = touchpad_write_i2c(client, cbc_0d_addr, &key_cbc_val, 1);
		if (rc) {
			pr_err("[touch] failed to read the 0D_CBC_val[0x%x]=0x%x, (rc=%d)\n",
				cbc_0d_addr, key_cbc_val, rc);
			return rc;
		}
		
		
		#if 0
		if (system_rev <= EVT1_3 && (msm8960_project_id == DETROIT)) {
			if ((int)g_tp->fw_version == SYNAP_DS4_3_0_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + f54_analog_ctrl41;
			else
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		} else {
			if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		}
		#endif
		if ((int)g_tp->fw_version == SYNAP_DS4_3_2_1_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_2_2_FW_VER || 
			(int)g_tp->fw_version == SYNAP_DS4_3_6_0_33_FW_VER || (int)g_tp->fw_version == SYNAP_DS4_3_6_0_34_FW_VER)
				noise_mitigation_ctrl_addr = g_tp->pdt_map.F54_control_base + DS4_3_2_F54_ANALOG_CTRL41;
		noise_mitigation_ctrl = 0x0;
		rc = touchpad_write_i2c(client, noise_mitigation_ctrl_addr, &noise_mitigation_ctrl, 1);
		if (rc) {
			pr_err("[touch] failed to enable noise mitigation ctrl value[0x%x]=0x%x, (rc=%d)\n",
				noise_mitigation_ctrl_addr, noise_mitigation_ctrl, rc);
			return rc;
		}
		DBG_REG("enable noise mitigation ctrl value[0x%x]=0x%x\n", noise_mitigation_ctrl_addr, noise_mitigation_ctrl);
		msleep(1);
	}
	
	
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	msleep(1);
    rc = touchpad_read_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return rc;
	}
	
	
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base+1, &interrupt_enable, 1);
	if (rc) {
		pr_err("[touch] failed to write the interrupt reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, interrupt_enable, rc);
		return rc;
	}
	
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_control_base+1, &irq_cmd, 1);
	if (rc) {
		pr_err("[touch] failed to read the interrupt reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_control_base+1, irq_cmd, rc);
		return rc;
	}
	return rc;
}	


static int touchpad_F54_RX_RX_short(struct synaptics_tp_t *g_tp)
{
	int rc = 0;
	int i,j,k=0;
	int count = 0;
	
	
	g_tp->report_type = F54_RX_RX_SHORT_7;	
	rc = touchpad_F54_testing(g_tp);
	if (rc) {
		pr_err("[touch] failed to test F54_RX_RX_short_7, (rc=%d)\n",
			rc);
		return rc;
	}
	mdelay(1);
	
	
	g_tp->report_type = F54_RX_RX_REPORT_17;	
	rc = touchpad_F54_testing(g_tp);
	if (rc) {
		pr_err("[touch] failed to test F54_RX_RX_short_17, (rc=%d)\n",
			rc);
		return rc;
	}
	mdelay(1);
    
	
	for (i = 0; i < g_tp->number_of_rx_electrodes; i++) {
		DBG_REG("rx_rx_data[%d]",i);
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			if (i < g_tp->number_of_tx_electrodes) {
				g_tp->rx_rx_imagearray[i][j] = g_tp->rx_rx_imagearray_7[k][j];
				DBG_REG(" %2d",(int)g_tp->rx_rx_imagearray[i][j]);
			}else {
				if (k == g_tp->number_of_tx_electrodes)
					k = 0;
				g_tp->rx_rx_imagearray[i][j] = g_tp->rx_rx_imagearray_17[k][j];
				DBG_REG("  %2d",(int)g_tp->rx_rx_imagearray[i][j]);
			}	
		}
		k = k + 1;
		
	}

	for (i = 0; i < g_tp->number_of_rx_electrodes; i++) {
       for (j = 0; j < g_tp->number_of_rx_electrodes; j++) {
			if (i == j){ 
			    if ((int)g_tp->rx_rx_imagearray[i][j] < F54_RX_RX_MAX_LIMIT && (int)g_tp->rx_rx_imagearray[i][j] > F54_RX_RX_MIN_LIMIT)
				{
					count += 0;
					pr_info("[touch] rx-rx[%d][%d]=%d", i,j,(int)g_tp->rx_rx_imagearray[i][j]);
					
				}else {
					count += 1;
					pr_info("[touch] rx-rx[%d][%d]=%d (fail)", i,j,(int)g_tp->rx_rx_imagearray[i][j]);
				}	
			}
		}
		printk("\n");
	}
	if (count == 0) {
		g_tp->F54_testing_flag += 0;
		pr_info("[touch] Rx-Rx short testing Pass.\n");
	} else {
		g_tp->F54_testing_flag += RX_RX_SHORT_ERR;
		pr_info("[touch] Rx-Rx short testing Fail.\n");
	}
	
	return rc;
}


static int tp_selftest_all(void *data, u64 *val)
{
	struct synaptics_tp_t *g_tp = (struct synaptics_tp_t *)data;
	int rc = 0;
	uint8_t cmd;

	pr_info("[touch] self-test all.\n");
	mutex_lock(&g_tp->mutex);
	
	
	g_tp->F54_testing_flag = 0;

	pr_info("[touch] fw version:%d\n", (int)g_tp->fw_version);
	pr_info("[touch] config id:0x%x\n", g_tp->config_id);
	pr_info("[touch] the number of Rx(%d)*TX(%d) channels\n",g_tp->number_of_rx_electrodes, g_tp->number_of_tx_electrodes);
	if (g_tp->number_of_rx_electrodes != NUM_OF_RX_ELECTRODES || g_tp->number_of_tx_electrodes != NUM_OF_TX_ELECTRODES) 
	{
		pr_info("[touch] the number of rx channels(= %d) or the number of tx channels(= %d) are abnormal. (rc=%d)\n",
			g_tp->number_of_rx_electrodes, g_tp->number_of_tx_electrodes, rc);
		g_tp->F54_testing_flag += TX_RX_CHANNEL_ERR;
		
		if(g_tp->F54_testing_flag == 0) 
		{
			pr_info("[touch] self test all pass.(test result=%d)\n",SELF_TEST_ALL_PASS);  
			*val = SELF_TEST_ALL_PASS;
		} else {
			pr_info("[touch] self test all Failed.(test result=%d)\n",g_tp->F54_testing_flag);
			*val = g_tp->F54_testing_flag;
		}
		mutex_unlock(&g_tp->mutex);
		return rc;
	} else {
		
		pr_info("[touch] Start the high resistance testing.\n");
		g_tp->report_type = F54_HIGH_RESISTANCE;	
		rc = touchpad_F54_testing(g_tp);
		if (rc) {
			pr_err("[touch] failed to test the high resistance, (rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
		
		
		pr_info("[touch] Start the TX-TX short testing.\n");
		g_tp->report_type = F54_TX_TX_SHORT;
		rc = touchpad_F54_testing(g_tp);
		if (rc) {
			pr_err("[touch] failed to test the Tx-Tx short, (rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
	
		
		pr_info("[touch] Start the raw capacitance testing.\n");
		g_tp->report_type = F54_RAW_CAPACITANCE;
		rc = touchpad_F54_testing(g_tp);
		if (rc) {
			pr_err("[touch] failed to test the raw capacitance, (rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
	
		
		pr_info("[touch] Start the RX-RX short testing.\n");
		rc = touchpad_F54_RX_RX_short(g_tp);
		if (rc) {
			pr_err("[touch] failed to test the Rx-Rx short, (rc=%d)\n",
				rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
		
		if(g_tp->F54_testing_flag == 0) 
		{
			pr_info("[touch] self test all pass.(test result=%d)\n",SELF_TEST_ALL_PASS);  
			*val = SELF_TEST_ALL_PASS;
		} else {
			pr_info("[touch] self test all Fail.(test result=%d)\n",g_tp->F54_testing_flag);
			*val = g_tp->F54_testing_flag;
		}
	
		
		cmd = 0x1;
		rc = touchpad_write_i2c(g_tp->client, g_tp->pdt_map.F01_command_base, &cmd, 1);
		if (rc) {
			pr_err("[touch] failed to set sw reset reg([0x%x]=0x%x), (rc=%d)\n",
				g_tp->pdt_map.F01_command_base, cmd, rc);
			mutex_unlock(&g_tp->mutex);
			return rc;
		}
		msleep(500);
	}
	
	
	mutex_unlock(&g_tp->mutex);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tp_selftest, tp_selftest_all, NULL, "%llu")

static ssize_t tp_read_device_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = container_of( dev, struct i2c_client, dev);
	int rc;
    uint8_t page_select = 0x0;
	uint8_t device_id_value;
	
	mutex_lock(&g_tp->mutex);
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		mutex_unlock(&g_tp->mutex);	
		return rc;
	}
	mdelay(1);
    
	
	rc = touchpad_read_i2c(client, g_tp->pdt_map.F01_query_base, &device_id_value, 1);
	if (rc) {
		pr_err("[touch] failed to write the device id reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_query_base, device_id_value, rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	DBG_REG("the device id reg[0x%x]=0x%x\n", g_tp->pdt_map.F01_query_base, device_id_value);
	
	if(device_id_value == 0x01)
	{
		mutex_unlock(&g_tp->mutex);
		return snprintf(buf, 11, "%d\n",device_id_value);
	}	
	else 
	{
		mutex_unlock(&g_tp->mutex);
		return -EINVAL;
	}	
}
static DEVICE_ATTR(device_id, 0444, tp_read_device_id, NULL);



static void synaptics_disable(struct device *dev)
{
	struct synaptics_tp_t *g_tp = dev_get_drvdata(dev);
	mutex_lock(&g_tp->mutex);
	if (g_tp->is_suspended) {
		mutex_unlock(&g_tp->mutex);
		return;
	}

	disable_irq(g_tp->irq);
	g_tp->is_suspended = 1;
	mutex_unlock(&g_tp->mutex);
	return;
}
static int dbg_tp_onoff_get(void *data, u64 *value)
{
	struct synaptics_tp_t *g_tp = (struct synaptics_tp_t *)data;
	int state = 0;
	DBG_REG("%s touch panel\n", g_tp->is_suspended ? "disable":"enable");
	if (g_tp->is_suspended == 1)
		state = 0;
	else if (g_tp->is_suspended == 0)
		state = 1;
		
	*value = state;
	return 0;
}

static int dbg_tp_onoff_set(void *data, u64 value)
{
	struct synaptics_tp_t *g_tp = (struct synaptics_tp_t *)data;

	if (value == 0) {
		synaptics_disable(&g_tp->client->dev);
		touchpad_power_on_device(g_tp, 0);
		pr_info("Power off touch panel!!\n");
	} else if (value == 1) {
		touchpad_power_on_device(g_tp, 1);
		if (g_tp->is_suspended)
		{
			mutex_lock(&g_tp->mutex);
			enable_irq(g_tp->irq);
			g_tp->is_suspended=0;
			mutex_unlock(&g_tp->mutex);
		}
		pr_info("Power on touch panel!!\n");
	} else {
		pr_err("[touch] %s: Error command(%lld)!!\n",
			__func__, value);
	}
	DBG_REG("%s touch panel\n", g_tp->is_suspended ? "disable":"enable");
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tp_onoff_fops, dbg_tp_onoff_get, dbg_tp_onoff_set, "%llu\n");

static ssize_t tp_read_fw_ver(struct device *dev, struct device_attribute *attr,char *buf)
{
	if (g_tp->fw_version!=0x00)
		return snprintf(buf, 11, "%d\n",g_tp->fw_version);
	else
		return -EINVAL;
}
static DEVICE_ATTR(tp_fw_ver, 0444, tp_read_fw_ver, NULL);


static ssize_t tp_read_config_id(struct device *dev, struct device_attribute *attr,char *buf)
{

	if(g_tp->config_id != 0x0) 
		return snprintf(buf, 11, "%d\n",g_tp->config_id);
	else
		return -EINVAL;
}
static DEVICE_ATTR(tp_config_id, 0444, tp_read_config_id, NULL);


static ssize_t tp_read_product_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = container_of( dev, struct i2c_client, dev);
	int rc = 0;
    uint8_t page_select;
	uint8_t pid_addr,pid_value[11];
	
	pid_value[10] = 0;
	mutex_lock(&g_tp->mutex);
	
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	mdelay(1);
	
	pid_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query11;
	rc = touchpad_read_i2c(g_tp->client, pid_addr, &pid_value[0], 10);
	if (rc < 0) {
		pr_err("[touch] Failed to read the product id reg([0x%x]=0x%x), (rc=%d)\n", 
			pid_addr, pid_value[0], rc);
		mutex_unlock(&g_tp->mutex);
		return INVALID_TP;
	}
	mutex_unlock(&g_tp->mutex);
	return snprintf(buf, 11, "%s\n",pid_value);
}
static DEVICE_ATTR(tp_product_id, 0444, tp_read_product_id, NULL);

static ssize_t tp_update_config_byfile_result(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", config_update_result);
}

static ssize_t tp_update_config_byfile(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	char fwname[128];

	
	config_update_result = 1;
	pr_info("[touch], %s called\n", __func__);
	pr_info("[touch], Touch config file name: %s\n", buf);

	pr_info("[touch], +++ config id before updating: \n");
	touchpad_read_config_id();
	memset(fwname, 0, sizeof(fwname));
	if(count > 128) {
		pr_err("[touch], File length exceed buffer size(128)!\n");
		return -1;
	}
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&g_tp->mutex);
	config_update_result = touchpad_update_config_bin(fwname);
	mutex_unlock(&g_tp->mutex);

	pr_info("[touch], --- config id after updating: \n");
	touchpad_read_config_id();

	if(config_update_result != 0)
		return -1;
	return count;
}

static DEVICE_ATTR(tp_update_config, 0644, tp_update_config_byfile_result, tp_update_config_byfile);

#define TOUCH_DRIVER_NAME	"tp_s3202_dbg"

static void __devinit touchpad_debugfs_init(struct synaptics_tp_t *g_tp)
{
	g_tp->dent = debugfs_create_dir(TOUCH_DRIVER_NAME, NULL);
	if (g_tp->dent) {


		debugfs_create_file("tp_onoff", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_onoff_fops);
		
		debugfs_create_file("tp_self_test", S_IRUGO | S_IWUGO, g_tp->dent, g_tp, &tp_selftest);
	} else {
		pr_err("[touch] %s: Failed to create debugfs dir.\n", __func__);
		debugfs_remove_recursive(g_tp->dent);
	}
}


static int touchpad_create_sys_entries(struct i2c_client *client)
{
	int ret=0;

	ret = device_create_file(&client->dev,&dev_attr_device_id);
	WARN_ON(ret);
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_tp_fw_ver);
	WARN_ON(ret);   
	if (ret)  return ret;	

	ret = device_create_file(&client->dev,&dev_attr_tp_config_id);
	WARN_ON(ret); 
	if (ret)  return ret;
	
	ret = device_create_file(&client->dev,&dev_attr_tp_product_id);
	WARN_ON(ret); 
	if (ret)  return ret;

	ret = device_create_file(&client->dev,&dev_attr_tp_update_config);
	WARN_ON(ret); 
	return ret;
}

static void synaptics_suspend(struct device *dev)
{
	struct synaptics_tp_t *g_tp = dev_get_drvdata(dev);
	struct i2c_client *client = g_tp->client;
	int rc = 0;
    uint8_t page_select;
	uint8_t sleep_mode;

#if defined(CONFIG_FB)
	pr_info("[touch] %s(): auto sleep! +++\n", __func__);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	pr_info("[touch] %s(): early suspend! +++\n", __func__);
#endif
	mutex_lock(&g_tp->mutex);
	if (g_tp->is_suspended) {
		mutex_unlock(&g_tp->mutex);
		return;
	}

	
	
	 
	
	
	disable_irq(g_tp->irq);
	
	g_tp->is_suspended = 1;
	
	
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return;
	}
	msleep(1);
	
	
	sleep_mode = 0x01;
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base, &sleep_mode, 1);
	if (rc) {
		pr_err("[touch] failed to write the F01_control_base into sleep mode, (rc=%d)\n", rc);
		return;
	}
	
	
#ifdef CONFIG_PM_LOG
	PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_TOUCH);
#endif 
	mutex_unlock(&g_tp->mutex);

#if defined(CONFIG_FB)
	pr_info("[touch] %s(): auto sleep! ---\n", __func__);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	pr_info("[touch] %s(): early suspend! ---\n", __func__);
#endif
	return;
	
}

static void synaptics_resume(struct device *dev)
{
	struct synaptics_tp_t *g_tp = dev_get_drvdata(dev);
	struct i2c_client *client = g_tp->client;
	int rc = 0;
	int i;
	uint8_t page_select;
	uint8_t sleep_mode;

	
#if defined(CONFIG_FB)
	pr_info("[touch] %s(): auto sleep! +++\n", __func__);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	pr_info("[touch] %s(): late resume! +++\n", __func__);
#endif
	mutex_lock(&g_tp->mutex);
    
	for (i = 0 ; i < MAX_TS_REPORT_POINTS ; i++)
	{
		g_tp->msg[i].prev_state = g_tp->msg[i].state;
		g_tp->msg[i].state = TS_RELEASE;
		if(g_tp->msg[i].coord.z != -1) {
			g_tp->msg[i].coord.z = 0;
		} 
	}
	touchpad_report_mt_protocol(g_tp);
	touchpad_report_coord(g_tp);
	
	
	page_select = 0x00;
	rc = touchpad_write_i2c(client, PAGE_SELECT_REGISTER, &page_select, 1);
	if (rc) {
		pr_err("[touch] failed to write the page table reg([0x%x]=0x%x), (rc=%d)\n",
			PAGE_SELECT_REGISTER, page_select, rc);
		return;
	}
	msleep(1);
	
	
	sleep_mode = 0x00;
	rc = touchpad_write_i2c(client, g_tp->pdt_map.F01_control_base, &sleep_mode, 1);
	if (rc) {
		pr_err("[touch] failed to write the active mode reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_control_base, sleep_mode, rc);
		return;
	}

	enable_irq(g_tp->irq);
	g_tp->is_suspended = 0;
	
	
#ifdef CONFIG_PM_LOG
	PM_LOG_EVENT(PM_LOG_ON, PM_LOG_TOUCH);
#endif	
	mutex_unlock(&g_tp->mutex);

#if defined(CONFIG_FB)
	pr_info("[touch] %s(): auto sleep! ---\n", __func__);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	pr_info("[touch] %s(): late resume! ---\n", __func__);
#endif
	return ;
	
}


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct synaptics_tp_t *g_tp =
		container_of(self, struct synaptics_tp_t, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		g_tp && g_tp->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			synaptics_disable(&g_tp->client->dev);
			synaptics_resume(&g_tp->client->dev);
		}
		else if (*blank == FB_BLANK_POWERDOWN)
			synaptics_suspend(&g_tp->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void synaptics_early_suspend(struct early_suspend *h)
{
	struct synaptics_tp_t *g_tp = container_of(h, 
		struct synaptics_tp_t, ts_early_suspend);

	synaptics_suspend(&g_tp->input_dev->dev);
}

static void synaptics_late_resume(struct early_suspend *h)
{
	struct synaptics_tp_t *g_tp = container_of(h, 
		struct synaptics_tp_t, ts_early_suspend);

	synaptics_resume(&g_tp->input_dev->dev);
}
#endif


static const struct dev_pm_ops synaptics_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = synaptics_suspend,
	.resume  = synaptics_resume,
#endif
};


static const struct i2c_device_id i2cSYNAPTICSTouch_idtable[] = {
	{ SYNAPTICS_TP_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, i2cSYNAPTICSTouch_idtable);


#ifdef CONFIG_OF
static struct of_device_id rmi4_match_table[] = {
	{ .compatible = "synaptics,rmi4",},
	{ },
};
#else
#define rmi4_match_table NULL
#endif

static struct i2c_driver i2c_touchpad_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SYNAPTICS_TP_NAME,
		.of_match_table = rmi4_match_table,

#ifdef CONFIG_PM
		.pm = &synaptics_ts_pm_ops,
#endif
	},
	.probe	 = touchpad_probe,
	.remove	 = __devexit_p(touchpad_remove),
	.id_table = i2cSYNAPTICSTouch_idtable,
};


static int proc_detect_tp_chip_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	uint8_t devid_val;
    int rc;
	
	mutex_lock(&g_tp->mutex);
	
	rc = touchpad_read_i2c(g_tp->client, g_tp->pdt_map.F01_query_base, &devid_val, 1);
	if (rc) {
		pr_err("[touch] failed to write the device id reg([0x%x]=0x%x), (rc=%d)\n",
			g_tp->pdt_map.F01_query_base, devid_val, rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	mutex_unlock(&g_tp->mutex);
	
	*eof = 1;
	return sprintf(page, "%u\n", devid_val);
}

static int proc_tp_config_id_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_tp->config_id);
}

static int proc_tp_fwver_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	
	return sprintf(page, "%u\n", g_tp->fw_version);
}

static int proc_tp_product_id_read(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	
	uint8_t pid_addr,pid_value[11];
	int rc = 0;
	
	mutex_lock(&g_tp->mutex);
	pid_addr = g_tp->pdt_map.F01_query_base + f01_rmi_query11;
	rc = touchpad_read_i2c(g_tp->client, pid_addr, &pid_value[0], 10);
	if (rc < 0) {
		pr_err("[touch] Failed to read the product id reg([0x%x]=0x%x), (rc=%d)\n", pid_addr, pid_value[0], rc);
		mutex_unlock(&g_tp->mutex);
		return rc;
	}
	mutex_unlock(&g_tp->mutex);
	*eof = 1;
	return sprintf(page, "%s\n", pid_value);
}


static int touchpad_create_procfs_entries(struct i2c_client *client)
{
	int ret=0;

	proc_entry = proc_mkdir("touchinfo", NULL);
	
	if (proc_entry == NULL) {
		pr_err("failed to create detect_panel entry\n");
		return -EPERM;
	} else {
		
		proc1_entry = create_proc_entry("tp_chip", S_IRUGO, proc_entry);
		if (proc1_entry == NULL) {
			pr_err("failed to create detect_panel entry\n");
			return -EPERM;
		}	
		proc1_entry->read_proc = proc_detect_tp_chip_read;
		
		proc2_entry = create_proc_entry("tp_config_id", S_IRUGO, proc_entry);
		if (proc2_entry == NULL) {
			pr_err("failed to create tp_config_id entry\n");
			return -EPERM;
		}	
		proc2_entry->read_proc = proc_tp_config_id_read;
		
		proc3_entry = create_proc_entry("tp_product_id", S_IRUGO, proc_entry);
		if (proc3_entry == NULL) {
			pr_err("failed to create tp_product_id entry\n");
			return -EPERM;
		}	
		proc3_entry->read_proc = proc_tp_product_id_read;
		
		proc4_entry = create_proc_entry("tp_fwver", S_IRUGO, proc_entry);
		if (proc4_entry == NULL) {
			pr_err("failed to create tp_fwver entry\n");
			return -EPERM;
		}	
		proc4_entry->read_proc = proc_tp_fwver_read;
	}
	return ret;
}

static int synaptics_rmi4_parse_dt(struct device *dev,
				struct synaptics_tp_platform_data_t  *rmi4_pdata)
{
	struct device_node *np = dev->of_node;
	
	int rc = 0;

	
			
	
	
	rmi4_pdata->gpio_rst = of_get_named_gpio_flags(np,
			"synaptics,reset-gpio", 0, &rmi4_pdata->reset_flags);
	rmi4_pdata->gpio_irq = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, &rmi4_pdata->irq_flags);

	return rc;
}

static int __devinit touchpad_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int      result = 0;
	struct   synaptics_tp_platform_data_t *pdata = client->dev.platform_data;
	
	
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(*pdata),
			GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		result = synaptics_rmi4_parse_dt(&client->dev, pdata);
		if (result)
			return result;
	} else {
		pdata = client->dev.platform_data;
	}
	
	
	
	g_tp = kzalloc(sizeof(struct synaptics_tp_t), GFP_KERNEL);
    if(!g_tp) {
        result = -ENOMEM;
        return result;
    }
    
	
    g_tp->gpio_irq = pdata->gpio_irq;
    g_tp->gpio_rst = pdata->gpio_rst;
    g_tp->irq = gpio_to_irq(g_tp->gpio_irq);
    g_tp->client = client;
    mutex_init(&g_tp->mutex);
	
	
	result = touchpad_setup_gpio(g_tp);
	if(result) {
		pr_err("[touch] Failed to setup gpio, (result = %d)\n", result);
		goto err_alloc_mem;
	}
	msleep(15);
	
	
    result = touchpad_power_on_device(g_tp, 1);
    if(result) {
    	pr_err("[touch] Unable to power device, (result = %d)\n", result);
		goto err_setup_gpio;
    }
	    
	
    result = touchpad_config_gpio(g_tp);
    if(result) {
        pr_err("[touch] Failed to config gpio\n" );
		goto err_power_device;
    }
	
	
	result = touchpad_detect_synaptics(g_tp);
    if(result) {
		pr_err("[touch] Failed to detect Synaptics S3202 touch IC, (result = %d)\n", result);
		goto err_power_device;
    }	else {
		
		
	}
	client->driver = &i2c_touchpad_driver;
	i2c_set_clientdata(client, g_tp);
	
	
    result = touchpad_register_input(&g_tp->input, pdata, client);
    if(result) {
    	pr_err("[touch] Failed to register input device (result = %d)\n", result);
		goto err_power_device;
    }
    input_set_drvdata(g_tp->input, g_tp);
	
	result = keyarray_register_input(&g_tp->keyarray_input, client);
    if(result) {
        pr_err("[key] Failed to register input device (result = %d)\n", result);
        goto err_register_touch_input;
    }
    input_set_drvdata(g_tp->keyarray_input, g_tp);
	
	result = request_threaded_irq(g_tp->irq, NULL, touchpad_irq_thread,
		IRQF_TRIGGER_LOW | IRQF_ONESHOT, "synaptics_ts", g_tp);
	if (result < 0) {
		pr_err("[touch] Failed to request irq(%d), (result = %d)\n", g_tp->irq, result);
		goto err_register_keyarray_input;
	}
	

#if defined(CONFIG_FB)
	g_tp->fb_notif.notifier_call = fb_notifier_callback;

	result = fb_register_client(&g_tp->fb_notif);
	if (result) {
		pr_err("[touch] Unable to register fb_notifier, (result = %d)\n", result);
	}		
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	g_tp->ts_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	g_tp->ts_early_suspend.suspend = synaptics_early_suspend;
	g_tp->ts_early_suspend.resume = synaptics_late_resume;
	register_early_suspend(&g_tp->ts_early_suspend);
#endif 
	
	
	touchpad_debugfs_init(g_tp);
	result = touchpad_create_sys_entries(client);
	
	
	
	result = touchpad_create_procfs_entries(client);
	
	pr_info("[touch] Start Probe %s\n",
		(result < 0) ? "FAIL" : "PASS");

	for (i = 0 ; i < MAX_TS_REPORT_POINTS ; i++)
	{
		g_tp->msg[i].coord.z = -1;
	}
	return 0;

err_register_keyarray_input:
    input_unregister_device(g_tp->keyarray_input);
    input_free_device(g_tp->keyarray_input);
    g_tp->keyarray_input = NULL;
err_register_touch_input:
    input_unregister_device(g_tp->input);
    input_free_device(g_tp->input);
    g_tp->input = NULL;
err_power_device:
	touchpad_power_on_device(g_tp, 0);	
err_setup_gpio:
	touchpad_release_gpio(g_tp);
err_alloc_mem:
	kfree(g_tp);
	result = -EFAULT;
	return result;
}	


static int __devexit touchpad_remove(struct i2c_client *client)
{
	
	struct synaptics_tp_t *g_tp = i2c_get_clientdata(client);
		
#if defined(CONFIG_FB)
	if (fb_unregister_client(&g_tp->fb_notif))
		pr_err("[touch] Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&g_ts->ts_early_suspend));
#endif

	
	free_irq(g_tp->irq, g_tp);

	input_unregister_device(g_tp->keyarray_input);
    input_free_device(g_tp->keyarray_input);
    g_tp->keyarray_input = NULL;
	
	input_unregister_device(g_tp->input);
    input_free_device(g_tp->input);
    g_tp->input = NULL;
	touchpad_release_gpio(g_tp);
	
	touchpad_power_on_device(g_tp, 0);
	mutex_destroy(&g_tp->mutex); 
	
	g_tp = NULL;
	kfree(g_tp);


	return 0;
}

static int __init touchpad_init(void)
{
	int rc = 0;
	
	pr_info("BootLog, +%s\n", __func__);
	
	i2c_touchpad_driver.driver.name = SYNAPTICS_TP_NAME;
	rc = i2c_add_driver(&i2c_touchpad_driver);
	if (rc) {
		pr_err("[touch] Synaptics S3202 touch driver registration failed\n");
		return rc;
	}
	pr_info("[touch] Synaptics S3202 touch driver initialize success!\n");
	pr_info("BootLog, -%s, rc=%d\n", __func__,rc);
    return rc;
}

module_init(touchpad_init);



static void __exit touchpad_exit(void)
{
    i2c_del_driver(&i2c_touchpad_driver);
    pr_info("[touch] SYNAPTICS S3202 touch driver exit success!\n");
}
module_exit(touchpad_exit);

MODULE_DESCRIPTION("SYNAPTICS RMI4 touchpad driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Emily Jiang");
MODULE_ALIAS("platform:SYNAPTICS_RMI4_touch");
