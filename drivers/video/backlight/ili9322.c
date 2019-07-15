/*
 * Driver for the ILI9322 LCD Controller
 *
 * Based on ILI9322 driver, copyright 2012 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

#define ILI9322_CHIP_ID     0x00
#define ILI9322_VCOM_AMP    0x01
#define ILI9322_VCOM_HIGH   0x02
#define ILI9322_VREG1OUT    0x03
#define ILI9322_RESET       0x04
#define ILI9322_POWER       0x05
#define ILI9322_ENTRY       0x06
#define ILI9322_VBP         0x07
#define ILI9322_HBP         0x08
#define ILI9322_POLARITY    0x09
#define ILI9322_DISPLAY     0x0a
#define ILI9322_DCDC        0x0c
#define ILI9322_DRIVING     0x0d
#define ILI9322_CONTRAST    0x0e
#define ILI9322_BRIGHT      0x0f
#define ILI9322_GAMMA1      0x10
#define ILI9322_GAMMA2      0x11
#define ILI9322_GAMMA3      0x12
#define ILI9322_GAMMA4      0x13
#define ILI9322_GAMMA5      0x14
#define ILI9322_GAMMA6      0x15
#define ILI9322_GAMMA7      0x16
#define ILI9322_GAMMA8      0x17
#define ILI9322_POWER2      0x30
#define ILI9322_OTP_PROGRAM 0x42
#define ILI9322_OTP_STATUS  0x43
#define ILI9322_OTP_KEY     0x44

struct ili9322_data {
	unsigned	        reset;
	struct spi_device   *spi;
	int			        state;
};

static uint8_t ili9322_seq_vcom[] = {
	ILI9322_VCOM_AMP, 0x14,
    ILI9322_VCOM_HIGH, 0x3a,
};

static uint8_t ili9322_seq_gamma[] = {
    ILI9322_GAMMA1, 0xa7,
    ILI9322_GAMMA2, 0x55,
    ILI9322_GAMMA3, 0x71,
    ILI9322_GAMMA4, 0x71,
    ILI9322_GAMMA5, 0x73,
    ILI9322_GAMMA6, 0x55,
    ILI9322_GAMMA7, 0x18,
    ILI9322_GAMMA8, 0x62,
};

static uint8_t ili9322_disp_on[] = {
    ILI9322_POWER2, 0x0d
};

static uint8_t ili9322_disp_off[] = {
    ILI9322_POWER2, 0x09
};

static uint8_t ili9322_power_off[] = {
    ILI9322_POWER, 0x00
};

static uint8_t ili9322_power_on[] = {
    ILI9322_POWER, 0xef
};


static int ili9322_spi_write_then_read(struct lcd_device *lcddev,
				uint8_t *txbuf, uint16_t txlen,
				uint8_t *rxbuf, uint16_t rxlen)
{
	struct ili9322_data *lcd = lcd_get_data(lcddev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u16 *local_txbuf = NULL;
	int ret = 0;

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	if (txlen) {
		xfer[0].len = txlen;
		xfer[0].bits_per_word = 8;
		xfer[0].tx_buf = txbuf;
		spi_message_add_tail(&xfer[0], &msg);
	}

	if (rxlen) {
		xfer[1].len = rxlen;
		xfer[1].bits_per_word = 8;
		xfer[1].rx_buf = rxbuf;
		spi_message_add_tail(&xfer[1], &msg);
	}

	ret = spi_sync(lcd->spi, &msg);
	if (ret < 0)
		dev_err(&lcddev->dev, "Couldn't send SPI data\n");

	if (txlen)
		kfree(local_txbuf);

	return ret;
}

static inline int ili9322_spi_write_byte(struct lcd_device *lcddev,
					uint8_t value)
{
	return ili9322_spi_write_then_read(lcddev, &value, 1, NULL, 0);
}

static inline int ili9322_spi_write_array(struct lcd_device *lcddev,
					uint8_t *value, uint8_t len)
{
    int i;
    // 2 bytes at a time
    for (i = 0; i < len; i+=2) {
        ili9322_spi_write_then_read(lcddev, &value[i], 2, NULL, 0);
    }
	//return ili9322_spi_write_then_read(lcddev, value, len, NULL, 0);
}

static int ili9322_enter_standby(struct lcd_device *lcddev)
{
	int ret;

    dev_info(&lcddev->dev, "Panel enter standby\n");

	ret = ili9322_spi_write_array(lcddev, ili9322_disp_off,
				ARRAY_SIZE(ili9322_disp_off));
	if (ret < 0)
		return ret;

	usleep_range(10000, 12000);

	ret = ili9322_spi_write_array(lcddev, ili9322_power_off,
				ARRAY_SIZE(ili9322_power_off));
	if (ret < 0)
		return ret;

	/*
	 * The controller needs 120ms when entering in sleep mode before we can
	 * send the command to go off sleep mode
	 */
	msleep(120);

	return 0;
}

static int ili9322_exit_standby(struct lcd_device *lcddev)
{
	int ret;

    dev_info(&lcddev->dev, "Panel exit standby\n");

	ret = ili9322_spi_write_array(lcddev, ili9322_power_on,
				ARRAY_SIZE(ili9322_power_on));
	if (ret < 0)
		return ret;

	/*
	 * The controller needs 120ms when exiting from sleep mode before we
	 * can send the command to enter in sleep mode
	 */
	msleep(120);

	ret = ili9322_spi_write_array(lcddev, ili9322_disp_on,
				ARRAY_SIZE(ili9322_disp_on));
	if (ret < 0)
		return ret;

	return 0;
}

static void ili9322_lcd_reset(struct lcd_device *lcddev)
{
	struct ili9322_data *lcd = lcd_get_data(lcddev);

	/* Reset the screen */
	gpio_set_value(lcd->reset, 1);
	usleep_range(10000, 12000);
	gpio_set_value(lcd->reset, 0);
	usleep_range(10000, 12000);
	gpio_set_value(lcd->reset, 1);

	/* The controller needs 120ms to recover from reset */
	msleep(120);
}

static int ili9322_lcd_init(struct lcd_device *lcddev)
{
	int ret;

    ret = ili9322_spi_write_array(lcddev, ili9322_seq_vcom,
				ARRAY_SIZE(ili9322_seq_vcom));
	if (ret < 0)
		return ret;

	ret = ili9322_spi_write_array(lcddev, ili9322_seq_gamma,
				ARRAY_SIZE(ili9322_seq_gamma));
	if (ret < 0)
		return ret;

   	ret = ili9322_exit_standby(lcddev);
	if (ret < 0)
		return ret;

	return 0;
}

#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

static int ili9322_set_power(struct lcd_device *lcddev, int power)
{
	struct ili9322_data *lcd = lcd_get_data(lcddev);
	int ret = 0;

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->state))
		ret = ili9322_exit_standby(lcddev);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->state))
		ret = ili9322_enter_standby(lcddev);

	if (ret == 0)
		lcd->state = power;
	else
		dev_warn(&lcddev->dev, "failed to set power mode %d\n", power);

	return ret;
}

static int ili9322_get_power(struct lcd_device *lcddev)
{
	struct ili9322_data *lcd = lcd_get_data(lcddev);

	return lcd->state;
}

static struct lcd_ops ili9322_ops = {
	.set_power	= ili9322_set_power,
	.get_power	= ili9322_get_power,
};

static const struct of_device_id ili9322_dt_ids[] = {
	{
		.compatible = "ilitek,ili9322",
		.data = ili9322_lcd_init,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ili9322_dt_ids);

static int ili9322_probe(struct spi_device *spi)
{
	struct lcd_device *lcddev;
	struct ili9322_data *lcd;
	const struct of_device_id *match;
	int ret;

	lcd = devm_kzalloc(&spi->dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI setup failed.\n");
		return ret;
	}

	lcd->spi = spi;

	match = of_match_device(ili9322_dt_ids, &spi->dev);
	if (!match || !match->data)
		return -EINVAL;

	lcd->reset = of_get_named_gpio(spi->dev.of_node, "gpios-reset", 0);
	if (!gpio_is_valid(lcd->reset)) {
		dev_err(&spi->dev, "Missing dt property: gpios-reset\n");
		return -EINVAL;
	}

	ret = devm_gpio_request_one(&spi->dev, lcd->reset,
				    GPIOF_OUT_INIT_HIGH,
				    "ili9322-reset");
	if (ret) {
		dev_err(&spi->dev,
			"failed to request gpio %d: %d\n",
			lcd->reset, ret);
		return -EINVAL;
	}

	lcddev = devm_lcd_device_register(&spi->dev, "mxsfb", &spi->dev, lcd,
					&ili9322_ops);
	if (IS_ERR(lcddev)) {
		ret = PTR_ERR(lcddev);
		return ret;
	}
	spi_set_drvdata(spi, lcddev);

	ili9322_lcd_reset(lcddev);

	ret = ((int (*)(struct lcd_device *))match->data)(lcddev);
	if (ret) {
		dev_err(&spi->dev, "Couldn't initialize panel\n");
		return ret;
	}

	dev_info(&spi->dev, "Panel probed\n");

	return 0;
}

static struct spi_driver ili9322_driver = {
	.probe  = ili9322_probe,
	.driver = {
		.name = "ili9322",
		.of_match_table = ili9322_dt_ids,
	},
};

module_spi_driver(ili9322_driver);

MODULE_AUTHOR("Wenting Zhang <zephray@outlook.com>");
MODULE_DESCRIPTION("ILITEK ILI9322 LCD Driver");
MODULE_LICENSE("GPL");
