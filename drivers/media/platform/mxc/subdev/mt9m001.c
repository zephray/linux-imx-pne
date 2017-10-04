/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define MIN_FPS 10
#define MAX_FPS 10
#define DEFAULT_FPS 10

#define MT9M_XCLK_MIN 20000000
#define MT9M_XCLK_MAX 20000000

enum mt9m_mode {
	mt9m_mode_MIN = 0,
	mt9m_mode_VGA_640_480 = 0,
	mt9m_mode_SXGA_1280_1024 = 1,
	mt9m_mode_MAX = 1
};

enum mt9m_frame_rate {
	mt9m_10_fps,
};

static int mt9m_framerates[] = {
	[mt9m_10_fps] = 10,
};

struct mt9m_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct reg_value {
	u8 u8RegAddr;
	u16 u16Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct mt9m_mode_info {
	enum mt9m_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct mt9m {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct mt9m_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct mt9m mt9m_data;

static struct reg_value mt9m_init_setting_VGA[] = {
	{0x01, 0x000C, 0, 0}, {0x02, 0x0014, 0, 0}, {0x03, 0x03BF, 0, 0}, 
	{0x04, 0x04FF, 0, 0}, {0x05, 0x0009, 0, 0}, {0x06, 0x0019, 0, 0}, 
	{0x07, 0x0002, 0, 0}, {0x09, 0x0200, 0, 0}, {0x0C, 0x0000, 0, 0}, 
	{0x1E, 0x8000, 0, 0}, {0x20, 0x111C, 0, 0}, {0x2B, 0x0008, 0, 0},
	{0x2C, 0x0008, 0, 0}, {0x2D, 0x0008, 0, 0}, {0x2E, 0x0008, 0, 0},
	{0x35, 0x0010, 0, 0}, {0x5F, 0x0904, 0, 0}, {0x60, 0x0000, 0, 0},
	{0x61, 0x0000, 0, 0}, {0x62, 0x0498, 0, 0}, {0x63, 0x0000, 0, 0}, 
	{0x64, 0x0000, 0, 0},  
};

static struct reg_value mt9m_setting_VGA[] = {
	{0x03, 0x03BF, 0, 0}, {0x09, 0x0200, 0, 0}, {0x20, 0x111C, 0, 0}, {0x35, 0x0010, 0, 0}, 
};

static struct reg_value mt9m_setting_SXGA[] = {
	{0x03, 0x03FF, 0, 0}, {0x09, 0x0419, 0, 0}, {0x20, 0x1104, 0, 0}, {0x35, 0x0008, 0, 0}, 
};

static struct mt9m_mode_info mt9m_mode_info_data[1][mt9m_mode_MAX + 1] = {
	{
		{mt9m_mode_VGA_640_480,      640,  480,
		mt9m_setting_VGA,
		ARRAY_SIZE(mt9m_setting_VGA)},
		{mt9m_mode_SXGA_1280_1024,   1280, 1024,
		mt9m_setting_SXGA,
		ARRAY_SIZE(mt9m_setting_SXGA)},
	},
};

static int mt9m_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int mt9m_remove(struct i2c_client *client);

static s32 mt9m_read_reg(u8 reg, u16 *val);
static s32 mt9m_write_reg(u8 reg, u16 val);

static const struct i2c_device_id mt9m_id[] = {
	{"mt9m", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mt9m_id);

static struct i2c_driver mt9m_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "mt9m",
		  },
	.probe  = mt9m_probe,
	.remove = mt9m_remove,
	.id_table = mt9m_id,
};

static const struct mt9m_datafmt mt9m_colour_fmts[] = {
	{MEDIA_BUS_FMT_Y8_1X8, V4L2_COLORSPACE_SRGB},
};


static struct mt9m *to_mt9m(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct mt9m, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct mt9m_datafmt
			*mt9m_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt9m_colour_fmts); i++)
		if (mt9m_colour_fmts[i].code == code)
			return mt9m_colour_fmts + i;

	return NULL;
}

static inline void mt9m_power_down(int enable)
{

}

static inline void mt9m_reset(void)
{

}

static int mt9m_regulator_enable(struct device *dev)
{
	return 0;
}

static s32 mt9m_write_reg(u8 reg, u16 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg;
	au8Buf[1] = val >> 8;
	au8Buf[2] = val & 0xff;	

	if (i2c_master_send(mt9m_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 mt9m_read_reg(u8 reg, u16 *val)
{
	u8 au8Reg = 0;
	u16 u16RdVal = 0;

	au8Reg = reg;

	if (1 != i2c_master_send(mt9m_data.i2c_client, &au8Reg, 1)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (2 != i2c_master_recv(mt9m_data.i2c_client, &u16RdVal, 2)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u16RdVal);
		return -1;
	}

	*val = u16RdVal;

	return u16RdVal;
}

static void mt9m_soft_reset(void)
{
	/* software reset */
	mt9m_write_reg(0x0D, 0x0001);

	/* delay at least 5ms */
	msleep(10);

	mt9m_write_reg(0x0D, 0x0000);
}

static int mt9m_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u8 RegAddr = 0;
	register u8 Mask = 0;
	register u16 Val = 0;
	u16 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u8RegAddr;
		Val = pModeSetting->u16Val;
		Mask = pModeSetting->u8Mask;

		retval = mt9m_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

static int mt9m_init_mode(void)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	mt9m_soft_reset();

	pModeSetting = mt9m_init_setting_VGA;
	ArySize = ARRAY_SIZE(mt9m_init_setting_VGA);
	retval = mt9m_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	/* skip 9 vysnc: start capture at 10th vsync */
	msleep(300);

	mt9m_data.pix.width = 640;
	mt9m_data.pix.height = 480;
err:
	return retval;
}

static int mt9m_change_mode_direct(enum mt9m_frame_rate frame_rate,
			    enum mt9m_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;

	if (mode > mt9m_mode_MAX || mode < mt9m_mode_MIN) {
		pr_err("Wrong mt9m mode detected!\n");
		return -1;
	}

	pModeSetting = mt9m_mode_info_data[frame_rate][mode].init_data_ptr;
	ArySize =
		mt9m_mode_info_data[frame_rate][mode].init_data_size;

	mt9m_data.pix.width = mt9m_mode_info_data[frame_rate][mode].width;
	mt9m_data.pix.height = mt9m_mode_info_data[frame_rate][mode].height;

	if (mt9m_data.pix.width == 0 || mt9m_data.pix.height == 0 ||
	    pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	/* set mt9m to subsampling mode */
	retval = mt9m_download_firmware(pModeSetting, ArySize);

	msleep(400);

	return retval;
}


static int mt9m_change_mode(enum mt9m_frame_rate frame_rate,
			    enum mt9m_mode mode)
{
	int retval = 0;

	if (mode > mt9m_mode_MAX || mode < mt9m_mode_MIN) {
		pr_err("Wrong mt9m mode detected!\n");
		return -1;
	}
	pr_info("Changing mt9m mode to %d\n", mode);
	retval = mt9m_change_mode_direct(frame_rate, mode);

	return retval;
}

/*!
 * mt9m_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int mt9m_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m *sensor = to_mt9m(client);

	if (on)
		clk_enable(mt9m_data.sensor_clk);
	else
		clk_disable(mt9m_data.sensor_clk);

	sensor->on = on;

	return 0;
}


/*!
 * mt9m_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int mt9m_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m *sensor = to_mt9m(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * mt9m_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int mt9m_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m *sensor = to_mt9m(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum mt9m_frame_rate frame_rate;
	int ret = 0;

	pr_info("mt9m_s_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:

		/* Actual frame rate we use */
		tgt_fps = 10;
		frame_rate = mt9m_10_fps;

		ret = mt9m_change_mode(frame_rate,
				a->parm.capture.capturemode);
		if (ret < 0)
			goto error;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

error:
	return ret;
}

static int mt9m_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct mt9m_datafmt *fmt = mt9m_find_datafmt(mf->code);

	if (!fmt) {
		mf->code	= mt9m_colour_fmts[0].code;
		mf->colorspace	= mt9m_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int mt9m_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m *sensor = to_mt9m(client);

	/* MIPI CSI could have changed the format, double-check */
	if (!mt9m_find_datafmt(mf->code))
		return -EINVAL;

	mt9m_try_fmt(sd, mf);
	sensor->fmt = mt9m_find_datafmt(mf->code);

	return 0;
}

static int mt9m_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m *sensor = to_mt9m(client);

	const struct mt9m_datafmt *fmt = sensor->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int mt9m_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   u32 *code)
{
	if (index >= ARRAY_SIZE(mt9m_colour_fmts))
		return -EINVAL;

	*code = mt9m_colour_fmts[index].code;
	return 0;
}

/*!
 * mt9m_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int mt9m_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > mt9m_mode_MAX)
		return -EINVAL;

	fse->max_width =
			max(mt9m_mode_info_data[0][fse->index].width,
			    mt9m_mode_info_data[1][fse->index].width);
	fse->min_width = fse->max_width;
	fse->max_height =
			max(mt9m_mode_info_data[0][fse->index].height,
			    mt9m_mode_info_data[1][fse->index].height);
	fse->min_height = fse->max_height;
	return 0;
}

/*!
 * mt9m_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int mt9m_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count;

	if (fie->index < 0 || fie->index > mt9m_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warning("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(mt9m_mode_info_data); i++) {
		for (j = 0; j < (mt9m_mode_MAX + 1); j++) {
			if (fie->width == mt9m_mode_info_data[i][j].width
			 && fie->height == mt9m_mode_info_data[i][j].height
			 && mt9m_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator =
						mt9m_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int mt9m_set_clk_rate(void)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = mt9m_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)MT9M_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)MT9M_XCLK_MIN);
	mt9m_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(mt9m_data.sensor_clk, mt9m_data.mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", mt9m_data.mclk);
	return ret;
}


/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(void)
{
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum mt9m_frame_rate frame_rate;
	int ret;

	mt9m_data.on = true;

	/* mclk */
	tgt_xclk = mt9m_data.mclk;

	/* Default camera frame rate is set in probe */
	tgt_fps = 10;
	frame_rate = mt9m_10_fps;

	ret = mt9m_init_mode();

	return ret;
}


static struct v4l2_subdev_video_ops mt9m_subdev_video_ops = {
	.g_parm = mt9m_g_parm,
	.s_parm = mt9m_s_parm,

	.s_mbus_fmt	= mt9m_s_fmt,
	.g_mbus_fmt	= mt9m_g_fmt,
	.try_mbus_fmt	= mt9m_try_fmt,
	.enum_mbus_fmt	= mt9m_enum_fmt,
};

static const struct v4l2_subdev_pad_ops mt9m_subdev_pad_ops = {
	.enum_frame_size       = mt9m_enum_framesizes,
	.enum_frame_interval   = mt9m_enum_frameintervals,
};

static struct v4l2_subdev_core_ops mt9m_subdev_core_ops = {
	.s_power	= mt9m_s_power,
};

static struct v4l2_subdev_ops mt9m_subdev_ops = {
	.core	= &mt9m_subdev_core_ops,
	.video	= &mt9m_subdev_video_ops,
	.pad	= &mt9m_subdev_pad_ops,
};



/*!
 * mt9m I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int mt9m_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u16 chip_id;

	/* mt9m pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* Set initial values for the sensor struct. */
	memset(&mt9m_data, 0, sizeof(mt9m_data));
	mt9m_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(mt9m_data.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(mt9m_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&mt9m_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(mt9m_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(mt9m_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	mt9m_set_clk_rate();

	clk_prepare_enable(mt9m_data.sensor_clk);

	mt9m_data.io_init = mt9m_reset;
	mt9m_data.i2c_client = client;
	mt9m_data.pix.pixelformat = V4L2_PIX_FMT_GREY;
	mt9m_data.pix.width = 640;
	mt9m_data.pix.height = 480;
	mt9m_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	mt9m_data.streamcap.capturemode = 0;
	mt9m_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	mt9m_data.streamcap.timeperframe.numerator = 1;

	mt9m_reset();

	chip_id = 0;
	retval = mt9m_read_reg(0x00, &chip_id);
	pr_info("camera id %04x\n", chip_id);
	if (chip_id != 0x3184) {
		pr_warning("camera mt9m001 is not found\n");
		return -ENODEV;
	}

	retval = init_device();
	if (retval < 0) {
		clk_disable_unprepare(mt9m_data.sensor_clk);
		pr_warning("camera mt9m init failed\n");
		mt9m_power_down(1);
		return retval;
	}

	clk_disable(mt9m_data.sensor_clk);

	v4l2_i2c_subdev_init(&mt9m_data.subdev, client, &mt9m_subdev_ops);

	retval = v4l2_async_register_subdev(&mt9m_data.subdev);
	if (retval < 0)
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);

	pr_info("camera mt9m is found\n");
	return retval;
}

/*!
 * mt9m I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int mt9m_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);

	clk_unprepare(mt9m_data.sensor_clk);

	return 0;
}

module_i2c_driver(mt9m_i2c_driver);

MODULE_AUTHOR("Wenting Zhang");
MODULE_DESCRIPTION("MT9M001 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
