/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/pinctrl/consumer.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_pinctrl_right_init(struct gf_dev* gf_dev)
{
	int ret = 0;
	struct device *dev = &gf_dev->spi->dev;

	gf_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(gf_dev->pinctrl)) {
		FP_LOG_ERROR("Target does not use pinctrl\n");
		ret = PTR_ERR(gf_dev->pinctrl);
		goto err;
	}

	gf_dev->gpio_state_active = pinctrl_lookup_state(gf_dev->pinctrl, "gf_fp_r_active");
	if (IS_ERR_OR_NULL(gf_dev->gpio_state_active)) {
		FP_LOG_ERROR("Cannot get active pinstate\n");
		ret = PTR_ERR(gf_dev->gpio_state_active);
		goto err;
	}

	gf_dev->gpio_state_suspend = pinctrl_lookup_state(gf_dev->pinctrl, "gf_fp_r_suspend");
	if (IS_ERR_OR_NULL(gf_dev->gpio_state_suspend)) {
		FP_LOG_ERROR("Cannot get sleep pinstate\n");
		ret = PTR_ERR(gf_dev->gpio_state_suspend);
		goto err;
	}
	FP_LOG_INFO("success\n");
	return 0;
err:
	gf_dev->pinctrl = NULL;
	gf_dev->gpio_state_active = NULL;
	gf_dev->gpio_state_suspend = NULL;
	return ret;
}

int gf_pinctrl_right_select(struct gf_dev* gf_dev, bool on)
{
	int ret = 0;
	struct pinctrl_state *pins_state;

	pins_state = on ? gf_dev->gpio_state_active : gf_dev->gpio_state_suspend;
	if (IS_ERR_OR_NULL(pins_state)) {
		FP_LOG_ERROR("not a valid '%s' pinstate\n",
			on ? "gf_fp_active" : "gf_fp_suspend");
		return -1;
	}

	ret = pinctrl_select_state(gf_dev->pinctrl, pins_state);
	if (ret) {
		FP_LOG_ERROR("can not set %s pins\n",
			on ? "gf_fp_active" : "gf_fp_suspend");
	}
	FP_LOG_INFO("success\n");
	return ret;
}
int gf_right_parse_dts(struct gf_dev* gf_dev)
{
	int rc = 0;
	/*get reset resource*/
/*	gf_dev->pwr_gpio_vddio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_pwr_vddio",0);
	if(!gpio_is_valid(gf_dev->pwr_gpio_vddio)) {
		FP_LOG_ERROR("RESET GPIO is invalid.\n");
		return -1;
	}
	FP_LOG_INFO("gf::1.8V pwr gpio:%d\n", gf_dev->pwr_gpio_vddio);
	rc = gpio_request(gf_dev->pwr_gpio_vddio, "goodix_pwr_vddio");
	if(rc) {
		FP_LOG_ERROR("Failed to request pwr GPIO. rc = %d\n", rc);
		return -1;
	}

	gpio_direction_output(gf_dev->pwr_gpio_vddio, 1);*/

	/*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,goodix_reset_r",0);
	if(!gpio_is_valid(gf_dev->reset_gpio)) {
		FP_LOG_ERROR("RESET GPIO is invalid.\n");
		return -1;
	}
	FP_LOG_INFO("gf::reset right gpio:%d\n", gf_dev->reset_gpio);
	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset_r");
	if(rc) {
		FP_LOG_ERROR("Failed to request RESET GPIO. rc = %d\n", rc);
		return -1;
	}

	gpio_direction_output(gf_dev->reset_gpio, 1);

	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_set_value(gf_dev->reset_gpio, 1);
	}
	//FP_LOG_INFO("1.8v power %s\n", gpio_get_value(gf_dev->pwr_gpio_vddio)? "on" : "off");
	/*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,goodix_irq_r",0);
	FP_LOG_INFO("gf::irq_right_gpio:%d\n", gf_dev->irq_gpio);
	if(!gpio_is_valid(gf_dev->irq_gpio)) {
		FP_LOG_ERROR("IRQ GPIO is invalid.\n");
		return -1;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq_r");
	if(rc) {
		FP_LOG_ERROR("Failed to request IRQ GPIO. rc = %d\n", rc);
		return -1;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	FP_LOG_INFO("goodix gf_parse_dts success\n");
	return 0;
}

void gf_right_cleanup(struct gf_dev	* gf_dev)
{
	FP_LOG_INFO("[info] %s\n",__func__);
	if (gpio_is_valid(gf_dev->irq_gpio))
	{
		gpio_free(gf_dev->irq_gpio);
		FP_LOG_INFO("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio))
	{
		gpio_free(gf_dev->reset_gpio);
		FP_LOG_INFO("remove reset_gpio success\n");
	}
}
/*
int gf_right_power_on(struct gf_dev* gf_dev)
{
	int rc = 0;
	if(!gf_dev) {
		FP_LOG_INFO("gf_dev null\n");
	}
	if (gpio_is_valid(gf_dev->pwr_gpio_vdd)) {
		gpio_set_value(gf_dev->pwr_gpio_vdd, 1);
	}
	if (gpio_is_valid(gf_dev->pwr_gpio_vddio)) {
		gpio_set_value(gf_dev->pwr_gpio_vddio, 1);
	}
	msleep(10);
	FP_LOG_INFO("right power on\n");

	return rc;
}

int gf_right_power_off(struct gf_dev* gf_dev)
{
	int rc = 0;
	if(!gf_dev) {
		FP_LOG_INFO("gf_dev null\n");
	}
	if (gpio_is_valid(gf_dev->pwr_gpio_vdd)) {
		gpio_set_value(gf_dev->pwr_gpio_vdd, 0);
	}
	if (gpio_is_valid(gf_dev->pwr_gpio_vddio)) {
		gpio_set_value(gf_dev->pwr_gpio_vddio, 0);
	}

	gf_right_cleanup(gf_dev); //free gpio resource

	FP_LOG_INFO("right power off\n");
	return rc;
}
*/
int gf_right_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if(gf_dev == NULL) {
		FP_LOG_INFO("Input buff is NULL.\n");
		return -1;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_right_irq_num(struct gf_dev *gf_dev)
{
	if(gf_dev == NULL) {
		FP_LOG_INFO("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

