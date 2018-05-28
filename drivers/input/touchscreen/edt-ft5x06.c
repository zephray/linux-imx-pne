/*
 * Copyright (C) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 * Daniel Wagener <daniel.wagener@kernelconcepts.de> (M09 firmware support)
 * Lothar Waßmann <LW@KARO-electronics.de> (DT support)
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a driver for the EDT "Polytouch" family of touch controllers
 * based on the FocalTech FT5x06 line of chips.
 *
 * Development of this driver has been sponsored by Glyn:
 *    http://www.glyn.com/Products/Displays
 */

#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/input/edt-ft5x06.h>

#define MAX_SUPPORT_POINTS		5

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

struct edt_ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	char name[20];

	int reset_pin;
	int irq_pin;
	int wake_pin;

	u8 addr;
	u8 data;

	struct mutex mutex;
};

static int edt_ft5x06_ts_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;

	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;

	return 0;
}

static irqreturn_t edt_ft5x06_ts_isr(int irq, void *dev_id)
{
	struct edt_ft5x06_ts_data *tsdata = dev_id;
	struct device *dev = &tsdata->client->dev;
	u8 cmd;
	u8 rdbuf[29];
	int i, type, x, y, id;
	int offset, tplen, datalen;
	int error;

	cmd = 0x02;
	offset = 1;
	tplen = 6;
	datalen = 29;

	memset(rdbuf, 0, sizeof(rdbuf));

	error = edt_ft5x06_ts_readwrite(tsdata->client,
					sizeof(cmd), &cmd,
					datalen, rdbuf);
	if (error) {
		dev_err_ratelimited(dev, "Unable to fetch data, error: %d\n",
				    error);
		goto out;
	}

	for (i = 0; i < MAX_SUPPORT_POINTS; i++) {
		u8 *buf = &rdbuf[i * tplen + offset];
		bool down;

		type = buf[0] >> 6;
		/* ignore Reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;
		id = (buf[2] >> 4) & 0x0f;
		down = type != TOUCH_EVENT_UP;

		dev_info(dev, "x = %d, y = %d, id = %d", x, y, id);

		input_mt_slot(tsdata->input, id);
		input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, down);

		if (!down)
			continue;

		input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);
	}

	input_mt_report_pointer_emulation(tsdata->input, true);
	input_sync(tsdata->input);

out:
	return IRQ_HANDLED;
}

static int edt_ft5x06_register_write(struct edt_ft5x06_ts_data *tsdata,
				     u8 addr, u8 value)
{
	u8 wrbuf[4];

	wrbuf[0] = addr;
	wrbuf[1] = value;

	return edt_ft5x06_ts_readwrite(tsdata->client, 2,
					wrbuf, 0, NULL);
}

static int edt_ft5x06_register_read(struct edt_ft5x06_ts_data *tsdata,
				    u8 addr)
{
	u8 wrbuf[2], rdbuf[2];
	int error;

	wrbuf[0] = addr;
	error = edt_ft5x06_ts_readwrite(tsdata->client, 1,
						wrbuf, 1, rdbuf);
	if (error)
		return error;

	return rdbuf[0];
}

struct edt_ft5x06_attribute {
	struct device_attribute dattr;
	size_t field_offset;
	u8 type;
};

#define EDT_ATTR(_field, _mode, _type)				\
	struct edt_ft5x06_attribute edt_ft5x06_attr_##_field = {	\
		.dattr = __ATTR(_field, _mode,				\
				edt_ft5x06_setting_show,		\
				edt_ft5x06_setting_store),		\
		.field_offset = offsetof(struct edt_ft5x06_ts_data, _field), \
		.type = _type,					\
	}

#define EDT_AT_ADDR 0
#define EDT_AT_DATA 1

static ssize_t edt_ft5x06_setting_show(struct device *dev,
				       struct device_attribute *dattr,
				       char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)tsdata + attr->field_offset;
	int val;
	size_t count = 0;
	int error = 0;
	u8 type;

	mutex_lock(&tsdata->mutex);

	type = attr->type;

	if (type == EDT_AT_DATA) {
		val = edt_ft5x06_register_read(tsdata, tsdata->addr);
		if (val < 0) {
			error = val;
			dev_err(&tsdata->client->dev,
				"Failed to fetch attribute %s, error %d\n",
				dattr->attr.name, error);
			goto out;
		}
		*field = val;
	}
	else {
		val = *field;
	}

	count = scnprintf(buf, PAGE_SIZE, "%d\n", val);
out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static ssize_t edt_ft5x06_setting_store(struct device *dev,
					struct device_attribute *dattr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	struct edt_ft5x06_attribute *attr =
			container_of(dattr, struct edt_ft5x06_attribute, dattr);
	u8 *field = (u8 *)tsdata + attr->field_offset;
	unsigned int val;
	int error;
	u8 type;

	mutex_lock(&tsdata->mutex);

	error = kstrtouint(buf, 0, &val);
	if (error)
		goto out;

	type = attr->type;

	if (type == EDT_AT_DATA) {
		error = edt_ft5x06_register_write(tsdata, tsdata->addr, val);
		if (error) {
			dev_err(&tsdata->client->dev,
				"Failed to update attribute %s, error: %d\n",
				dattr->attr.name, error);
			goto out;
		}
	}
	*field = val;

out:
	mutex_unlock(&tsdata->mutex);
	return error ?: count;
}

static EDT_ATTR(addr, S_IWUSR | S_IRUGO, 0);
static EDT_ATTR(data, S_IWUSR | S_IRUGO, 1);

static struct attribute *edt_ft5x06_attrs[] = {
	&edt_ft5x06_attr_addr.dattr.attr,
	&edt_ft5x06_attr_data.dattr.attr,
	NULL
};

static const struct attribute_group edt_ft5x06_attr_group = {
	.attrs = edt_ft5x06_attrs,
};

static inline void
edt_ft5x06_ts_prepare_debugfs(struct edt_ft5x06_ts_data *tsdata,
			      const char *debugfs_name)
{
}

static inline void
edt_ft5x06_ts_teardown_debugfs(struct edt_ft5x06_ts_data *tsdata)
{
}

static int edt_ft5x06_ts_reset(struct i2c_client *client,
			struct edt_ft5x06_ts_data *tsdata)
{
	int error;

	if (gpio_is_valid(tsdata->wake_pin)) {
		error = devm_gpio_request_one(&client->dev,
					tsdata->wake_pin, GPIOF_OUT_INIT_LOW,
					"edt-ft5x06 wake");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as wake pin, error %d\n",
				tsdata->wake_pin, error);
			return error;
		}

		msleep(5);
		gpio_set_value(tsdata->wake_pin, 1);
	}
	if (gpio_is_valid(tsdata->reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = devm_gpio_request_one(&client->dev,
					tsdata->reset_pin, GPIOF_OUT_INIT_LOW,
					"edt-ft5x06 reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				tsdata->reset_pin, error);
			return error;
		}

		msleep(5);
		gpio_set_value(tsdata->reset_pin, 1);
		msleep(300);
	}

	return 0;
}

static int edt_ft5x06_ts_identify(struct i2c_client *client,
					struct edt_ft5x06_ts_data *tsdata)
{
	u8 rdbuf[1];
	//char *p;
	int error;
	char *model_name = tsdata->name;

	error = edt_ft5x06_ts_readwrite(client, 1, "\xA8", 1, rdbuf);
	if (error)
		return error;

	snprintf(model_name, 20, "generic ft5x06 (%02x)", rdbuf[0]);

	return 0;
}

#define EDT_ATTR_CHECKSET(name, reg) \
do {								\
	edt_ft5x06_register_write(tsdata, reg, pdata->name);	\
} while (0)

#define EDT_GET_PROP(name, reg) {				\
	u32 val;						\
	if (of_property_read_u32(np, #name, &val) == 0)		\
		edt_ft5x06_register_write(tsdata, reg, val);	\
}

#ifdef CONFIG_OF
static int edt_ft5x06_i2c_ts_probe_dt(struct device *dev,
				struct edt_ft5x06_ts_data *tsdata)
{
	struct device_node *np = dev->of_node;

	/*
	 * irq_pin is not needed for DT setup.
	 * irq is associated via 'interrupts' property in DT
	 */
	tsdata->irq_pin = -EINVAL;
	tsdata->reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	tsdata->wake_pin = of_get_named_gpio(np, "wake-gpios", 0);

	return 0;
}
#else
static inline int edt_ft5x06_i2c_ts_probe_dt(struct device *dev,
					struct edt_ft5x06_ts_data *tsdata)
{
	return -ENODEV;
}
#endif

static int edt_ft5x06_ts_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	const struct edt_ft5x06_platform_data *pdata =
						dev_get_platdata(&client->dev);
	struct edt_ft5x06_ts_data *tsdata;
	struct input_dev *input;
	int error;

	dev_dbg(&client->dev, "probing for EDT FT5x06 I2C\n");

	tsdata = devm_kzalloc(&client->dev, sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	if (!pdata) {
		error = edt_ft5x06_i2c_ts_probe_dt(&client->dev, tsdata);
		if (error) {
			dev_err(&client->dev,
				"DT probe failed and no platform data present\n");
			return error;
		}
	} else {
		tsdata->reset_pin = pdata->reset_pin;
		tsdata->irq_pin = pdata->irq_pin;
		tsdata->wake_pin = -EINVAL;
	}

	error = edt_ft5x06_ts_reset(client, tsdata);
	if (error)
		return error;

	if (gpio_is_valid(tsdata->irq_pin)) {
		error = devm_gpio_request_one(&client->dev, tsdata->irq_pin,
					GPIOF_IN, "edt-ft5x06 irq");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				tsdata->irq_pin, error);
			return error;
		}
	}

	input = devm_input_allocate_device(&client->dev);
	if (!input) {
		dev_err(&client->dev, "failed to allocate input device.\n");
		return -ENOMEM;
	}

	mutex_init(&tsdata->mutex);
	tsdata->client = client;
	tsdata->input = input;

	error = edt_ft5x06_ts_identify(client, tsdata);
	if (error) {
		dev_err(&client->dev, "touchscreen probe failed\n");
		return error;
	}

	input->name = tsdata->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_set_abs_params(input, ABS_X, 0, 640 - 1, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 480 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, 640 - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, 480 * 64 - 1, 0, 0);

	if (!pdata)
		touchscreen_parse_of_params(input);

	error = input_mt_init_slots(input, MAX_SUPPORT_POINTS, 0);
	if (error) {
		dev_err(&client->dev, "Unable to init MT slots.\n");
		return error;
	}

	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					edt_ft5x06_ts_isr,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		return error;
	}

	error = sysfs_create_group(&client->dev.kobj, &edt_ft5x06_attr_group);
	if (error)
		return error;

	error = input_register_device(input);
	if (error)
		goto err_remove_attrs;

	edt_ft5x06_ts_prepare_debugfs(tsdata, dev_driver_string(&client->dev));
	device_init_wakeup(&client->dev, 1);

	dev_dbg(&client->dev,
		"EDT FT5x06 initialized: IRQ %d, WAKE pin %d, Reset pin %d.\n",
		client->irq, tsdata->wake_pin, tsdata->reset_pin);

	return 0;

err_remove_attrs:
	sysfs_remove_group(&client->dev.kobj, &edt_ft5x06_attr_group);
	return error;
}

static int edt_ft5x06_ts_remove(struct i2c_client *client)
{
	struct edt_ft5x06_ts_data *tsdata = i2c_get_clientdata(client);

	edt_ft5x06_ts_teardown_debugfs(tsdata);
	sysfs_remove_group(&client->dev.kobj, &edt_ft5x06_attr_group);

	return 0;
}

static int __maybe_unused edt_ft5x06_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused edt_ft5x06_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(edt_ft5x06_ts_pm_ops,
			 edt_ft5x06_ts_suspend, edt_ft5x06_ts_resume);

static const struct i2c_device_id edt_ft5x06_ts_id[] = {
	{ "edt-ft5x06", 0, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, edt_ft5x06_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id edt_ft5x06_of_match[] = {
	{ .compatible = "edt,edt-ft5206", },
	{ .compatible = "edt,edt-ft5306", },
	{ .compatible = "edt,edt-ft5406", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, edt_ft5x06_of_match);
#endif

static struct i2c_driver edt_ft5x06_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "edt_ft5x06",
		.of_match_table = of_match_ptr(edt_ft5x06_of_match),
		.pm = &edt_ft5x06_ts_pm_ops,
	},
	.id_table = edt_ft5x06_ts_id,
	.probe    = edt_ft5x06_ts_probe,
	.remove   = edt_ft5x06_ts_remove,
};

module_i2c_driver(edt_ft5x06_ts_driver);

MODULE_AUTHOR("Simon Budig <simon.budig@kernelconcepts.de>");
MODULE_DESCRIPTION("EDT FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
