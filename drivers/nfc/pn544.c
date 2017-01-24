/*
 * Driver for the PN544 NFC chip.
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Jari Vanhala <ext-jari.vanhala@nokia.com>
 * Contact: Matti Aaltonen <matti.j.aaltonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/completion.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nfc/pn544.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/serial_core.h> /* for TCGETS */
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <asm/atomic.h> 
#include <linux/seq_file.h> 


#include <mach/socinfo.h>


#ifdef CONFIG_PM_LOG
#include <mach/pm_log.h>
#endif  

#define DRIVER_CARD	"PN544 NFC"
#define DRIVER_DESC	"NFC driver for PN544"

static struct of_device_id msm_match_table[] = {
	{.compatible = "nxp,pn544"},
	{ }
};
MODULE_DEVICE_TABLE(of, msm_match_table);

#define HCI_MODE	0
#define FW_MODE		1

enum pn544_state {
	PN544_ST_COLD,
	PN544_ST_FW_READY,
	PN544_ST_READY,
};

enum pn544_irq {
	PN544_NONE,
	PN544_INT,
};

struct pn544_info {
	struct miscdevice miscdev;
	struct i2c_client *i2c_dev;


	struct regulator_bulk_data regs[1];
	enum pn544_state state;
	wait_queue_head_t read_wait;
	loff_t read_offset;
	enum pn544_irq read_irq;
	struct mutex read_mutex; /* Serialize read_irq access */
	struct mutex mutex; /* Serialize info struct access */
	spinlock_t irq_enabled_lock;
	u8 *buf;
	size_t buflen;
	bool irq_enabled;
	int irq_gpio;
	int ven_gpio;
	int firmware_gpio;
	int ven_polarity;
};

static const char reg_vdd_io[]	= "nfc_ven"; 
static const char reg_vbat[]	= "nfc_bat"; 
static const char reg_vsim[]	= "nfc_pmvcc"; 


static u8 dev_i2c_response = 0;

static atomic_t antenna_sleftest_status = ATOMIC_INIT(0);
static u8 force_antenna_test = 0;


static struct regulator_bulk_data nfc_regs[] = {
	{ .supply = reg_vdd_io,	.min_uV = 1800000,	.max_uV = 1800000 },

};


static char test_addr_result = 0;
static ssize_t pn544_i2c_test_set(struct device *dev,
			  struct device_attribute *attr,  const char *buf, size_t count)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_dev;
	char test_addr = PN544_I2C_ADDR;
	int ret = 0;

	ret = i2c_master_send(client, &test_addr, 1);
	pr_info("--%s: send(0x%x) to pn544, ret(%d)--\n", __func__, test_addr, ret);

	ret = i2c_master_recv(client, &test_addr_result, 1);
	pr_info("--%s: recv(0x%x) from pn544, ret(%d)--\n", __func__, test_addr_result, ret);

	return count;
}


static ssize_t pn544_i2c_test_get(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	
	return snprintf(buf, PAGE_SIZE, "%d,%d\n", dev_i2c_response, test_addr_result);
}

static int pn544_enable(struct pn544_info *info, int mode)
{
	int r;

	r = regulator_bulk_enable(ARRAY_SIZE(info->regs), info->regs);
	if (r < 0)
		return r;

	
#ifdef CONFIG_PM_LOG	
	PM_LOG_EVENT(PM_LOG_ON, PM_LOG_NFC);
#endif  

    gpio_set_value(info->firmware_gpio, mode ? 1 : 0);
	msleep(PN544_GPIO4VEN_TIME);
	gpio_set_value(info->ven_gpio, info->ven_polarity ? 0 : 1);

	info->read_irq = PN544_NONE;

	if (mode) {
		info->state = PN544_ST_FW_READY;
		pr_info("%s: now in FW-mode\n", __func__);
	} else {
		info->state = PN544_ST_READY;
		pr_info("%s: now in HCI-mode\n", __func__);
	}

	usleep_range(10000, 15000);

	return 0;
}

static void pn544_disable(struct pn544_info *info)
{
	
#ifdef CONFIG_PM_LOG
	PM_LOG_EVENT(PM_LOG_OFF, PM_LOG_NFC);
#endif  

	gpio_set_value(info->ven_gpio, info->ven_polarity ? 1 : 0);

	pr_info("%s: now in OFF-mode\n", __func__);

	msleep(PN544_RESETVEN_TIME);

	regulator_bulk_disable(ARRAY_SIZE(info->regs), info->regs);
}

static void pn544_disable_irq(struct pn544_info *pn544_dev)
{
	unsigned long flags = 0;;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->i2c_dev->irq);
		pn544_dev->irq_enabled = false;
	}

	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_info *pn544_dev = dev_id;

	pr_debug( "[nfcdb]++ %s ++\n", __func__);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		pr_debug("[nfcdb]%s: irq release!\n", __func__);
		return IRQ_HANDLED;
	}

	pn544_disable_irq(pn544_dev);

	
	wake_up(&pn544_dev->read_wait);

	pr_debug( "[nfcdb]-- %s --\n", __func__);
	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
			  size_t count, loff_t *offset)
{

	struct pn544_info *pn544_dev = filp->private_data;
	char tmp[PN544_MAX_BUFFER_SIZE];
	int ret, i, retries = 0;

	if (count > PN544_MAX_BUFFER_SIZE)
		count = PN544_MAX_BUFFER_SIZE;

	pr_debug("[nfcinfo]%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->i2c_dev->irq);
		ret = wait_event_interruptible(pn544_dev->read_wait,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);
		if (ret)
			goto fail;
	}

	
	do {
		ret = i2c_master_recv(pn544_dev->i2c_dev, tmp, count);
		if (ret < 0)
			msleep_interruptible(5);
	} while ((ret < 0) && (++retries < 5));

	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
		}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
		}

	if (copy_to_user(buf, tmp, ret)) {
		pr_err("%s: failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("[nfcdb]%s: IFD->PC:", __func__);
	for(i = 0; i < ret; i++)
		pr_debug(" %02X", tmp[i]);

	pr_debug("\n");




	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{

	struct pn544_info  *pn544_dev;
	char tmp[PN544_MAX_BUFFER_SIZE];
	int ret, i, retries = 0;

	pn544_dev = filp->private_data;

	if (count > PN544_MAX_BUFFER_SIZE)
		count = PN544_MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s: failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("[nfcinfo]%s : writing %zu bytes.\n", __func__, count);

	
	do {
		ret = i2c_master_send(pn544_dev->i2c_dev, tmp, count);
		if (ret < 0)
			msleep_interruptible(5);
 	} while ((ret < 0) && (++retries < 5));


	if (ret != count || ret < 0) {
		pr_err("%s: i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}

	pr_debug("[nfcdb]%s: PC->IFD:", __func__);

	for(i = 0; i < count; i++)
		pr_debug(" %02X", tmp[i]);

	pr_debug("\n");




	return ret;
}

static long pn544_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pn544_info *info = file->private_data;
	int r = 0;

	pr_debug("[nfcdb]%s: info: %p, cmd: 0x%x\n", __func__, info, cmd);

	mutex_lock(&info->mutex);

	switch (cmd) {
	case PN544_SET_PWR:
		pr_info("%s: PN544_SET_PWR_MODE\n", __func__);
 		if (arg == 2) {
			
			pr_info("%s: power on with firmware update.\n", __func__);
 			pn544_disable(info);
			pn544_enable(info, FW_MODE);
		} else if (arg == 1) {
			
			pr_info("%s: normal power on\n", __func__);
			pn544_enable(info, HCI_MODE);
		} else if (arg == 0) {
			
			pr_info("%s: power off\n", __func__);
			pn544_disable(info);
		} else {
			pr_err("%s: the pwr set error...", __func__);
			r = -EINVAL;
			goto out;
		}
		break;
 	default:
		pr_err("%s: The IOCTL isn't exist!!\n", __func__);
		r = -EINVAL;
		break;
	}
out:
	mutex_unlock(&info->mutex);
	return r;
}

static int pn544_open(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data,
					       struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;
	int r = 0;

	pr_debug("[nfcinfo]%s: info: %p, client %p\n", __func__, info, client);

	file->private_data = info;

	mutex_lock(&info->mutex);
	r = pn544_enable(info, HCI_MODE);
	mutex_unlock(&info->mutex);

	return r;
}

static int pn544_close(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data,
					       struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;

	pr_debug("[nfcinfo]%s: info: %p, client %p\n", __func__, info, client);

	mutex_lock(&info->mutex);
	pn544_disable(info);
	mutex_unlock(&info->mutex);

	return 0;
}

static const struct file_operations pn544_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= pn544_dev_read,
	.write		= pn544_dev_write,
	.open		= pn544_open,
	.release	= pn544_close,
	.unlocked_ioctl	= pn544_ioctl,
};

#ifdef CONFIG_PM
static int pn544_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_info *info = i2c_get_clientdata(client);
	int r = 0;

	pr_debug("[nfcinfo]%s: info: %p, client %p\n", __func__, info, client);
	
	if (!dev_i2c_response)
		return 0;
	
	mutex_lock(&info->mutex);

	switch (info->state) {
	case PN544_ST_FW_READY:
		/* Do not suspend while upgrading FW, please! */
		r = -EPERM;
		break;

	case PN544_ST_READY:
		/*
		 * CHECK: Device should be in standby-mode. No way to check?
		 * Allowing low power mode for the regulator is potentially
		 * dangerous if pn544 does not go to suspension.
		 */
		break;

	case PN544_ST_COLD:
		break;
	};

	mutex_unlock(&info->mutex);
	return r;
}

static int pn544_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_info *info = i2c_get_clientdata(client);
	int r = 0;

	pr_debug("[nfcinfo]%s: %p, client %p\n", __func__, info, client);

	mutex_lock(&info->mutex);

	switch (info->state) {
	case PN544_ST_READY:
		/*
		 * CHECK: If regulator low power mode is allowed in
		 * pn544_suspend, we should go back to normal mode
		 * here.
		 */
		break;

	case PN544_ST_COLD:
		break;

	case PN544_ST_FW_READY:
		break;
	};

	mutex_unlock(&info->mutex);

	return r;
}

static SIMPLE_DEV_PM_OPS(pn544_pm_ops, pn544_suspend, pn544_resume);
#endif

static struct device_attribute pn544_attr[] = {
	
	__ATTR(i2c_test, (S_IWUSR | S_IWGRP | S_IRUGO), pn544_i2c_test_get, pn544_i2c_test_set),
};


#ifdef CONFIG_DEBUG_FS
static struct dentry *dent;


static int pn544_host_int_gpio_get(void *data, u64 *value)
{
	int gpio;
	
	if (socinfo_get_oem_board_id() > EVT2){
		gpio = gpio_get_value(PN544_HOST_INT_GPIO);
	}else{
		gpio = gpio_get_value(PN544_ENABLE_GPIO);
	}

	pr_debug("[nfcdbfs]%s: host_int_gpio is %s!!\n", __func__, gpio? "high":"low");

	*value = gpio;

	return 0;
}

static int pn544_host_int_gpio_set(void *data, u64 value)
{
	if(value == 1)
	{
		
		if (socinfo_get_oem_board_id() > EVT2){
			gpio_set_value(PN544_HOST_INT_GPIO, 1);
		}else{
			gpio_set_value(PN544_ENABLE_GPIO, 1);
		}
		pr_debug("[nfcdbfs]%s: set host_int_gpio to be high!!\n", __func__);
	}
	else if(value == 0)
	{
		
		if (socinfo_get_oem_board_id() > EVT2){
			gpio_set_value(PN544_HOST_INT_GPIO, 0);
		}else{
			gpio_set_value(PN544_ENABLE_GPIO, 0);
		}
		pr_debug("[nfcdbfs]%s: set host_int_gpio to be low!!\n", __func__);
	}
	else
		pr_debug("[nfcdbfs]%s: Unsupport value!!\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_host_int_gpio_fops, pn544_host_int_gpio_get, pn544_host_int_gpio_set, "%llu\n");


static int pn544_enable_gpio_get(void *data, u64 *value)
{
	int gpio;
	

	if (socinfo_get_oem_board_id() > EVT2){
		gpio = gpio_get_value(PN544_ENABLE_GPIO);
	}else{
		gpio = gpio_get_value(PN544_HOST_INT_GPIO);
	}

	pr_debug("[nfcdbfs]%s: enable_gpio is %s!!\n", __func__, gpio? "high":"low");

	*value = gpio;

	return 0;
}

static int pn544_enable_gpio_set(void *data, u64 value)
{
	if(value == 1)
	{
		
		if (socinfo_get_oem_board_id() > EVT2){
			gpio_set_value(PN544_ENABLE_GPIO, 1);
		}else{
			gpio_set_value(PN544_HOST_INT_GPIO, 1);
		}
		pr_debug("[nfcdbfs]%s: set enable_gpio to be high!!\n", __func__);
	}
	else if(value == 0)
	{
		
		if (socinfo_get_oem_board_id() > EVT2){
			gpio_set_value(PN544_ENABLE_GPIO, 0);
		}else{
			gpio_set_value(PN544_HOST_INT_GPIO, 0);
		}
		pr_debug("[nfcdbfs]%s: set enable_gpio to be low!!\n", __func__);
	}
	else
		pr_debug("[nfcdbfs]%s: Unsupport value!!\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_enable_gpio_fops, pn544_enable_gpio_get, pn544_enable_gpio_set, "%llu\n");


static int pn544_fw_reset_get(void *data, u64 *value)
{
	int gpio;

	gpio = gpio_get_value(PN544_FW_RESET_GPIO);

	pr_debug("[nfcdbfs]%s: fw_reset_gpio_fops is %s!!\n", __func__, gpio? "high":"low");

	*value = gpio;

	return 0;
}

static int pn544_fw_reset_set(void *data, u64 value)
{
	if(value == 1)
	{
		gpio_set_value(PN544_FW_RESET_GPIO, 1);
		pr_debug("[nfcdbfs]%s: set fw_reset_gpio_fops to be high!!\n", __func__);
	}
	else if(value == 0)
	{
		gpio_set_value(PN544_FW_RESET_GPIO, 0);
		pr_debug("[nfcdbfs]%s: set fw_reset_gpio_fops to be low!!\n", __func__);
	}
	else
		pr_debug("[nfcdbfs]%s: Unsupport value!!\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_fw_reset_gpio_fops, pn544_fw_reset_get, pn544_fw_reset_set, "%llu\n");



static int pn544_dev_i2c_response_get(void *data, u64 *value)
{
	pr_debug("[nfcdbfs]%s: dev_i2c_response is %d\n", __func__, dev_i2c_response);

	*value = dev_i2c_response;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pn544_dev_i2c_response_fops, pn544_dev_i2c_response_get, NULL, "%llu\n");

#ifdef CONFIG_PN544_TEST

static int pn544_read_chip_id_open(struct inode *inode, struct file *file)
{
	pr_debug("[nfcdb]-- %s --\n", __func__);

	file->private_data = inode->i_private;

	return 0;
}

static int pn544_read_chip_id_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	struct i2c_client *client = info->i2c_dev;
	u16 chip_id = 0;
	int ret = 0, copy_size = 0;
	char buf[64];

	pr_debug("[nfcdb]-- %s --\n", __func__);

	
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(r_ver, client, &pn544_test_info);
		pn544_disable (info);

		chip_id = (dev_i2c_response << 8) | pn544_test_info.chip_version.sw_ver.value_byte[2];

		pr_debug("[nfcdbfs]%s: recv chip id[0x%x] from pn544.\n",
			__func__, chip_id);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "%d,", dev_i2c_response);
		ret += snprintf(buf + ret, copy_size - ret, "%d\n", pn544_test_info.chip_version.sw_ver.value_byte[2]);

		pr_debug("[nfcdbfs]%s: ret = %d, buf(%s)\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_read_chip_id_fops = {
	.open		= pn544_read_chip_id_open,
	.read		= pn544_read_chip_id_read,
};


static int pn544_read_sw_version_open(struct inode *inode, struct file *file)
{
	pr_debug("[nfcdb]-- %s --\n", __func__);

	file->private_data = inode->i_private;

	return 0;
}

static int pn544_read_sw_version_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	struct i2c_client *client = info->i2c_dev;
	int ret = 0, copy_size = 0;
	char buf[64];

	pr_debug("[nfcdb]-- %s --\n", __func__);

	
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(r_ver, client, &pn544_test_info);
		pn544_disable (info);

		pr_debug("[nfcdbfs]%s: recv sw version[0x%x] from pn544.\n",
			__func__,  pn544_test_info.chip_version.sw_ver.value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "chip-%d", pn544_test_info.chip_version.sw_ver.value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "-%d\n", pn544_test_info.chip_version.sw_ver.value_byte[0]);

		pr_debug("[nfcdbfs]%s: ret = %d, buf(%s)\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_read_sw_version_fops = {
	.open		= pn544_read_sw_version_open,
	.read		= pn544_read_sw_version_read,
};



static int pn544_antenna_selftset_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pr_debug("[nfctest]%s: start nfc antenna selftest process...\n", __func__);

	atomic_set(&antenna_sleftest_status, 1);

	return 0;
}

static int pn544_antenna_selftset_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	int ret = 0, copy_size = 0;
	char buf[64];

	pr_debug("[nfctest]%s: count = %Zd\n", __func__, count);

	
	if (*ppos == 0) {
			if (atomic_read(&antenna_sleftest_status) != 0 && force_antenna_test == 0) {
				pr_debug("[nfctest]%s: Need to write tolerance to do antenna selftest first!\n",
					__func__);
				return -EBUSY;
			}

		#if 0
			if (count >= sizeof(pn544_test_info.antenna_selftest_threshold)) {
				pr_debug("[nfctest]%s: Read byte is too big!\n",
					__func__);
				return -EINVAL;
			}
		#endif

			pn544_disable(info);
			pn544_enable(info, HCI_MODE);
			PN544_NFC_TEST(antenna_self_test, info->i2c_dev, &pn544_test_info);
			pn544_disable (info);
			force_antenna_test = 0;

		pr_debug("[nfcdbfs]%s: test result is [0x%08x]\n",
			__func__, pn544_test_info.antenna_selftest_threshold.value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "%d,", pn544_test_info.antenna_selftest_threshold.value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%d,", pn544_test_info.antenna_selftest_threshold.value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "%d,", pn544_test_info.antenna_selftest_threshold.value_byte[2]);
		ret += snprintf(buf + ret, copy_size - ret, "%d\n", pn544_test_info.antenna_selftest_threshold.value_byte[3]);

		pr_debug("[nfcdbfs]%s: ret = %d, buf(%s)\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static ssize_t pn544_antenna_selftset_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret, i = 0;
	char *buf = (char *) __get_free_page(GFP_USER);
	char *tmp = NULL;

	pr_debug("[nfctest]-- %s --\n", __func__);

	if (!buf) {
		pr_debug("[nfcdbfs]%s: can't get free GFP_USER page!\n", __func__);
		ret = -ENOMEM;
		atomic_set(&antenna_sleftest_status, 0);
		goto out;
	}

	if (count >= PAGE_SIZE) {
		pr_debug("[nfcdbfs]%s: count(%d) is bigger than PAGE_SIZE(%ld)\n",
			__func__, count, PAGE_SIZE);
		ret = -EINVAL;
		atomic_set(&antenna_sleftest_status, 0);
		goto out;
	}

	ret = copy_from_user(buf, buffer, count);
	if (ret) {
		pr_debug("[nfcdbfs]%s: copy_from_user failed, ret(%d)\n",
			__func__, ret);
		ret = -EFAULT;
		atomic_set(&antenna_sleftest_status, 0);
		goto out;
	}

	ret = count;
	buf[count] = '\0';

	tmp = buf;
	while (i < 5) {
		if (i < 4) {
			pn544_test_info.antenna_loop_tolerance.value_byte[i] =
				simple_strtol(tmp, NULL, 0);
		} else {

			force_antenna_test = simple_strtol(tmp, NULL, 0);
		}
		tmp = strchr(tmp, ',');
		if (tmp == NULL)
			break;

		tmp++;
		i++;
	}

	pr_debug("[nfctest]%s: tolerance is [%08x]\n",
		__func__, pn544_test_info.antenna_loop_tolerance.value);

	atomic_set(&antenna_sleftest_status, 0);

 out:
	free_page((unsigned long)buf);
	return ret;
}

static const struct file_operations pn544_antenna_sleftest_operations = {
	.open		= pn544_antenna_selftset_open,
	.read		= pn544_antenna_selftset_read,
	.write		= pn544_antenna_selftset_write,
};


static int pn544_swp_selftset_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pr_debug("[nfctest]%s: start nfc swp selftest process...\n", __func__);

	return 0;
}

static int pn544_swp_selftset_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	int ret = 0, copy_size = 0;
	char buf[64];

	pr_debug("[nfctest]%s: count = %Zd\n", __func__, count);

	
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(swp_self_test, info->i2c_dev, &pn544_test_info);
		pn544_disable (info);

		pr_debug("[nfcdbfs]%s: test result is [0x%08x]\n",
			__func__, pn544_test_info.swp_selftest_result.value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "%d,", pn544_test_info.swp_selftest_result.value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%d\n", pn544_test_info.swp_selftest_result.value_byte[1]);

		pr_debug("[nfcdbfs]%s: ret = %d, buf(%s)\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_swp_sleftest_operations = {
	.open		= pn544_swp_selftset_open,
	.read		= pn544_swp_selftset_read,
};


static int pn544_switchTx_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pr_debug("[nfctest]%s: start nfc switch Tx on process...\n", __func__);

	return 0;
}

static int pn544_switchTx_read(struct file *file, __user char *buffer, size_t count,
	loff_t *ppos)
{
	int ret = 0, copy_size = 0;
	char buf[64];

	pr_debug("[nfctest]%s: count = %Zd\n", __func__, count);

	
	if (*ppos == 0) {
		copy_size = sizeof(buf);
		ret = snprintf(buf, copy_size - ret, "%d\n", pn544_test_info.switch_Tx_onoff);
		pr_debug("[nfctest]%s: ret = %d, buf(%s)\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static ssize_t pn544_switchTx_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int ret, value;
	char *buf = (char *) __get_free_page(GFP_USER);
	struct pn544_info *info = file->private_data;

	pr_debug("[nfctest]-- %s --\n", __func__);

	if (!buf)
		return -ENOMEM;

	ret = -EINVAL;
	if (count >= PAGE_SIZE)
		goto out;

	ret = -EFAULT;
	if (copy_from_user(buf, buffer, count))
		goto out;

	ret = count;
	buf[count] = '\0';

	value = simple_strtol(buf, NULL, 10) ? 1 : 0;
	if (value == 1) {
		pn544_test_info.switch_Tx_onoff = 1;
		pr_debug("[nfctest]%s: switch Tx on...\n", __func__);
	} else {
		pn544_test_info.switch_Tx_onoff = 0;
		pr_debug("[nfctest]%s: switch Tx off...\n", __func__);
	}

	pn544_disable(info);
	pn544_enable(info, HCI_MODE);
	PN544_NFC_TEST(switch_Tx_only, info->i2c_dev, &pn544_test_info);

 out:
	free_page((unsigned long)buf);
	return ret;
}

static const struct file_operations pn544_switchTx_operations = {
	.open		= pn544_switchTx_open,
	.read		= pn544_switchTx_read,
	.write		= pn544_switchTx_write,
};


static int pn544_hci_TypeA_reader_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	pr_debug("[nfctest]%s: start nfc TypeA reader process...\n", __func__);

	return 0;
}

static int pn544_hci_TypeA_reader_read(struct file *file, __user char *buffer, size_t count,
			loff_t *ppos)
{
	struct pn544_info *info = file->private_data;
	int ret = 0, copy_size = 0;
	char buf[64];

	pr_debug("[nfctest]%s: count = %Zd\n", __func__, count);

	
	if (*ppos == 0) {
		pn544_disable(info);
		pn544_enable(info, HCI_MODE);
		PN544_NFC_TEST(hci_TypeA_reader, info->i2c_dev, &pn544_test_info);
		pn544_disable (info);


		pr_debug("[nfctest]%s: test result: uid_led[%d], uid[0x%08x 0x%08x]\n",
			__func__, pn544_test_info.card_data.uid_len,
			pn544_test_info.card_data.uid[0].value, pn544_test_info.card_data.uid[1].value);

		copy_size = sizeof(buf);

		ret = snprintf(buf, copy_size - ret, "len=%x,", pn544_test_info.card_data.uid_len);
		ret += snprintf(buf + ret, copy_size - ret, "uid=%x ", pn544_test_info.card_data.uid[0].value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[0].value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[0].value_byte[2]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[0].value_byte[3]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[1].value_byte[0]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[1].value_byte[1]);
		ret += snprintf(buf + ret, copy_size - ret, "%x ", pn544_test_info.card_data.uid[1].value_byte[2]);
		ret += snprintf(buf + ret, copy_size - ret, "%x\n", pn544_test_info.card_data.uid[1].value_byte[3]);

		pr_debug("[nfctest]%s: ret = %d, buf(%s)\n",
			__func__, ret, buf);
	}

	return simple_read_from_buffer(buffer, count, ppos, buf, ret);
}

static const struct file_operations pn544_TypeA_reader_operations = {
	.open		= pn544_hci_TypeA_reader_open,
	.read		= pn544_hci_TypeA_reader_read,
};
#endif


static void pn544_create_debugfs_entries(struct pn544_info *info)
{
	pr_debug("[nfcdb]-- %s --\n", __func__);

	dent = debugfs_create_dir(PN544_DRIVER_NAME, NULL);
	if (dent) {
	
	#ifdef CONFIG_PN544_TEST
		debugfs_create_file("read_chip_id", S_IRUGO, dent, info, &pn544_read_chip_id_fops);
		debugfs_create_file("read_sw_version", S_IRUGO, dent, info, &pn544_read_sw_version_fops);
		debugfs_create_file("TypeA_reader", S_IRUGO, dent, info, &pn544_TypeA_reader_operations);
		debugfs_create_file("swp_self_test", S_IRUGO, dent, info, &pn544_swp_sleftest_operations);

		debugfs_create_file("antenna_self_test", S_IRUGO | S_IWUGO, dent, info, &pn544_antenna_sleftest_operations);
		debugfs_create_file("switch_Tx", S_IWUGO, dent, info, &pn544_switchTx_operations);
	#endif

		debugfs_create_file("host_int_gpio", S_IRUGO | S_IWUGO, dent, info, &pn544_host_int_gpio_fops);
		debugfs_create_file("enable_gpio", S_IRUGO | S_IWUGO, dent, info, &pn544_enable_gpio_fops);
		debugfs_create_file("reset_gpio", S_IRUGO | S_IWUGO, dent, info, &pn544_fw_reset_gpio_fops);
		
		debugfs_create_file("dev_i2c_response", S_IRUGO | S_IWUGO, dent, info, &pn544_dev_i2c_response_fops);
	}
}

static void pn544_destroy_debugfs_entries(void)
{
	pr_debug("[nfcdb]-- %s --\n", __func__);

	if (dent)
		debugfs_remove_recursive(dent);
}
#endif

static int nfc_parse_dt(struct device *dev, struct pn544_nfc_platform_data *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;

	r = of_property_read_u32(np, "reg", &pdata->reg);
	if (r) {
		pr_err("%s: Parse reg failed, r(%d)\n", __func__, r);
		r = -EINVAL;
		goto err_parse;
	}
		
	if (socinfo_get_oem_board_id() > EVT2){
		pdata->irq_gpio = of_get_named_gpio(np, NFC_HOST_INT_GPIO, 0);
	}else{
		pdata->irq_gpio = of_get_named_gpio(np, NFC_ENABLE_GPIO, 0);
	}
	if ((!gpio_is_valid(pdata->irq_gpio))) {
		pr_err("%s: Parse irq gpio failed, gpio(%d)\n", __func__, pdata->irq_gpio);
		r = -EINVAL;
		goto err_parse;
	}
	if (socinfo_get_oem_board_id() > EVT2){
		pdata->ven_gpio = of_get_named_gpio(np, NFC_ENABLE_GPIO, 0);
	}else{
		pdata->ven_gpio = of_get_named_gpio(np, NFC_HOST_INT_GPIO, 0);
	}
	if ((!gpio_is_valid(pdata->ven_gpio))) {
		pr_err("%s: Parse ven gpio failed, gpio(%d)\n", __func__, pdata->ven_gpio);
		r = -EINVAL;
		goto err_parse;
	}

	pdata->firmware_gpio = of_get_named_gpio(np, NFC_FW_RESET_GPIO, 0);
	if ((!gpio_is_valid(pdata->firmware_gpio))) {
		pr_err("%s: Parse firmware gpio failed, gpio(%d)\n", __func__, pdata->firmware_gpio);
		r = -EINVAL;
		goto err_parse;
	}

	r = of_property_read_u32(np, "nxp,ven-polarity", &pdata->ven_polarity);
	if (r) {
		pr_err("%s: Parse ven-polarity failed, r(%d)\n", __func__, r);
		r = -EINVAL;
		goto err_parse;
	}

err_parse:
	return r;
}

static int __devinit pn544_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct pn544_info *info;
	struct pn544_nfc_platform_data *pdata;
	int i = 0, r = 0;
	
	char test_addr = PN544_I2C_ADDR;

	pr_info("%s: Probing pn544 driver\n", __func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct pn544_nfc_platform_data), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Cannot allocate memory for pn544_info.\n", __func__);
			r = -ENOMEM;
			goto err_parse_dt;
		}
		r = nfc_parse_dt(&client->dev, pdata);
		if (r)
			goto err_pdata_alloc;
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		pr_err("%s: No platform data.\n", __func__);
		r = -EINVAL;
		goto err_pdata_alloc;
	}
	pr_info("%s: reg(0x%x),irq_gpio(%d),ven_gpio(%d),fw_gpio(%d),ven_polarity(%d)\n",
		__func__, pdata->reg, pdata->irq_gpio, pdata->ven_gpio, pdata->firmware_gpio,
		pdata->ven_polarity);

	pr_info("%s: inside nxp-pn544 flags = %x\n", __func__, client->flags);

	if (pdata == NULL) {
		pr_err("%s: nfc probe failed.\n", __func__);
		r = -ENODEV;
		goto err_pdata_alloc;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		r = -ENODEV;
		goto err_pdata_alloc;
	}

	
	info = kzalloc(sizeof(struct pn544_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: Cannot allocate memory for pn544_info.\n", __func__);
		r = -ENOMEM;
		goto err_pdata_alloc;
	}

	
	if (gpio_is_valid(pdata->irq_gpio)) {
			
		if (socinfo_get_oem_board_id() > EVT2){
			r = gpio_request(pdata->irq_gpio, NFC_HOST_INT_GPIO);
		}else{
			r = gpio_request(pdata->irq_gpio, NFC_ENABLE_GPIO);
		}
		if (r) {
			pr_err("%s: Request NFC INT GPIO failed (%d)\n", __func__, r);
			goto err_info_alloc;
		}

		r = gpio_direction_input(pdata->irq_gpio);
		if (r) {
			pr_err("%s: Set GPIO Direction failed (%d)\n", __func__, r);
			goto err_irq_gpio;
		}

		r = gpio_to_irq(pdata->irq_gpio);
		if (r < 0) {
			pr_err("%s: Get IRQ from NFC INT GPIO failed (%d)\n", __func__, r);
			goto err_irq_gpio;
		}
		client->irq = r;
	} else {
		pr_err("%s: irq gpio not provided\n", __func__);
		goto err_info_alloc;
	}

	
	if (gpio_is_valid(pdata->ven_gpio)) {
		
		if (socinfo_get_oem_board_id() > EVT2){
			r = gpio_request(pdata->ven_gpio, NFC_ENABLE_GPIO);
		}else{
			r = gpio_request(pdata->ven_gpio, NFC_HOST_INT_GPIO);
		}
			
		if (r) {
			pr_err("%s: Request for NFC Enable GPIO failed (%d)\n", __func__, r);
			goto err_irq_gpio;
		}

		r = gpio_direction_output(pdata->ven_gpio, pdata->ven_polarity ? 1 : 0); 
		if (r) {
			pr_err("%s: Set GPIO Direction failed (%d)\n", __func__, r);
			goto err_ven_gpio;
		}
	} else {
		pr_err("%s: ven gpio not provided\n", __func__);
		goto err_irq_gpio;
	}

	
	if (gpio_is_valid(pdata->firmware_gpio)) {
		r = gpio_request(pdata->firmware_gpio, NFC_FW_RESET_GPIO);
		if (r) {
			pr_err("%s: Request for NFC FW Reset GPIO failed (%d)\n", __func__, r);
			goto err_ven_gpio;
		}

		r = gpio_direction_output(pdata->firmware_gpio, 0);
		if (r) {
			pr_err("%s: Set GPIO Direction failed (%d)\n", __func__, r);
			goto err_fw_gpio;
		}
	} else {
		pr_err("%s: firmware gpio not provided\n", __func__);
		goto err_ven_gpio;
	}

	info->irq_gpio = pdata->irq_gpio;
	info->ven_gpio = pdata->ven_gpio;
	info->firmware_gpio = pdata->firmware_gpio;
	info->ven_polarity = pdata->ven_polarity;

	info->buflen = max(PN544_MSG_MAX_SIZE, PN544_MAX_I2C_TRANSFER);
	info->buf = kzalloc(info->buflen, GFP_KERNEL);
	if (!info->buf) {
		pr_err("%s: Cannot allocate memory for pn544_info->buf.\n", __func__);
		r = -ENOMEM;
		goto err_fw_gpio;
	}


	info->regs[0] = nfc_regs[0];



	r = regulator_bulk_get(&client->dev, ARRAY_SIZE(info->regs),
				 info->regs);
	if (r < 0)
		goto err_kmalloc;

	r = regulator_bulk_set_voltage(ARRAY_SIZE(info->regs), info->regs);
	if (r < 0)
		goto err_bulk_reg;

	r = regulator_bulk_enable(ARRAY_SIZE(info->regs), info->regs);
	if (r < 0)
		goto err_bulk_reg;

	info->i2c_dev = client;
	mutex_init(&info->read_mutex);
	mutex_init(&info->mutex);
	init_waitqueue_head(&info->read_wait);
	spin_lock_init(&info->irq_enabled_lock);
	i2c_set_clientdata(client, info);

	
	pn544_disable(info);
	pn544_enable(info, HCI_MODE);

	
	do {
		r = i2c_master_send(client, &test_addr, 1);
		pr_info("%s: (%d) send(0x%x) to pn544, ret(%d)\n",
			__func__, i, test_addr, r);

		r = i2c_master_recv(client, &dev_i2c_response, 1);
		pr_info("%s: (%d) recv(0x%x) from pn544, ret(%d)\n",
			__func__, i, dev_i2c_response, r);

		if (dev_i2c_response == ((test_addr << 1) + 1))
			break;
		else {
			i++;
			
			usleep_range(200000, 250000);
		}

		if (i >= 5) {
			dev_i2c_response = 0;
			pr_err("%s: No nfc device exist!!\n", __func__);
		
		
		}
	} while (i < 5);

	pr_info("%s: misc devices setting....\n", __func__);
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = PN544_DRIVER_NAME;
	info->miscdev.fops = &pn544_fops;
	info->miscdev.parent = &client->dev;

	r = misc_register(&info->miscdev);
	if (r < 0) {
		pr_err("%s: Device registration failed\n", __func__);
		goto err_bulk_reg;
	}

	pr_info("%s: IRQ setting....\n", __func__);
	info->irq_enabled = true;
	info->irq_gpio = pdata->irq_gpio;

	r = request_irq(client->irq, pn544_dev_irq_handler,
					IRQF_TRIGGER_HIGH, PN544_DRIVER_NAME, info);
	if (r < 0) {
		pr_err("%s: Unable to register IRQ handler\n", __func__);
		goto err_micsc_reg;
	}

	pn544_disable_irq(info);

	
	for (i = 0; i < ARRAY_SIZE(pn544_attr); i++) {
		r = device_create_file(&client->dev, &pn544_attr[i]);
		if (r) {
			pr_err("%s: sysfs registration failed, error %d\n", __func__, r);
			goto err_irq;
		}
	}

	
	pn544_create_debugfs_entries(info);

	pn544_disable(info);
	pn544_enable(info, HCI_MODE);


#ifdef CONFIG_PN544_TEST
	memset(&pn544_test_info, 0, sizeof(struct chip_test_data_t));

#if 0 
	
	pr_debug("[nfctest]%s: start nfc version getting...");
	PN544_NFC_TEST(r_ver, client, &pn544_test_info);
	pn544_disable (info);

	pr_info("%s success: nfc sw version is %d-%d\n", __func__,
		pn544_test_info.chip_version.sw_ver.value_byte[1],
		pn544_test_info.chip_version.sw_ver.value_byte[0]);

	return 0;
#endif
#endif

	pn544_disable (info);

	pr_info("%s: success!!\n", __func__);

	return 0;

err_irq:
	free_irq(client->irq, info);
err_micsc_reg:
	misc_deregister(&info->miscdev);
err_bulk_reg:
	regulator_bulk_free(ARRAY_SIZE(info->regs), info->regs);
err_kmalloc:
	kfree(info->buf);
err_fw_gpio:
	gpio_free(pdata->firmware_gpio);
err_ven_gpio:
	gpio_free(pdata->ven_gpio);
err_irq_gpio:
	gpio_free(pdata->irq_gpio);
err_info_alloc:
	kfree(info);
err_pdata_alloc:
	devm_kfree(&client->dev, pdata);
err_parse_dt:
	return r;
}

static __devexit int pn544_remove(struct i2c_client *client)
{
	struct pn544_info *info = i2c_get_clientdata(client);

	int i;

	misc_deregister(&info->miscdev);

	for (i = 0; i < ARRAY_SIZE(pn544_attr); i++)
		device_remove_file(&client->dev, &pn544_attr[i]);

	
	pn544_destroy_debugfs_entries();

	if (info->state != PN544_ST_COLD) {
		gpio_set_value(info->ven_gpio, info->ven_polarity ? 1 : 0);

		info->read_irq = PN544_NONE;
	}

	free_irq(client->irq, info);
    gpio_free(info->irq_gpio);
	gpio_free(info->ven_gpio);
	gpio_free(info->firmware_gpio);

	regulator_bulk_free(ARRAY_SIZE(info->regs), info->regs);

	kfree(info->buf);
	kfree(info);

	pr_info("%s remove success!\n", __func__);

	return 0;
}

static const struct i2c_device_id pn544_id_table[] = {
	{PN544_DRIVER_NAME, 0},
	{ }
};

static struct i2c_driver pn544_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = PN544_DRIVER_NAME,
		.of_match_table = msm_match_table,
#ifdef CONFIG_PM
		.pm = &pn544_pm_ops,
#endif
	},
	.probe = pn544_probe,
	.id_table = pn544_id_table,
	.remove = __devexit_p(pn544_remove),
};

static int __init pn544_init(void)
{
	int r;

	r = i2c_add_driver(&pn544_driver);
	if (r) {
		pr_err(PN544_DRIVER_NAME ": driver registration failed\n");
		return r;
	}
	pr_info("%s initialize success!\n", __func__);

	return 0;
}

static void __exit pn544_exit(void)
{
	i2c_del_driver(&pn544_driver);
	pr_info("%s exit success!\n", __func__);
}

module_init(pn544_init);
module_exit(pn544_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
