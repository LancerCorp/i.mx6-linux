#include <linux/delay.h>
#include <linux/edm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <mach/common.h>
#include <mach/devices-common.h>
#include <mach/gpio.h>



static struct i2c_board_info pca9633_i2c_data[] __initdata = {
	{
		I2C_BOARD_INFO("pca9633", 0x62),
	},
};

static struct i2c_board_info etti_touch_i2c_data[] __initdata = {
	{
		I2C_BOARD_INFO("egalax_ts", 0x04),
	},
};

static __init int lancer_init(void) {
	i2c_register_board_info(edm_i2c[0], &pca9633_i2c_data[0], 1);
	gpio_direction_input(edm_external_gpio[5]);
	//etti_touch_i2c_data[0].irq = gpio_to_irq(edm_external_gpio[5]);
	//i2c_register_board_info(edm_i2c[1], &etti_touch_i2c_data[0], 1);
	gpio_direction_input(edm_external_gpio[6]);
	gpio_direction_input(edm_external_gpio[8]);
	gpio_direction_input(edm_external_gpio[9]);
	gpio_direction_input(IMX_GPIO_NR(3, 17));
	gpio_direction_input(IMX_GPIO_NR(3, 18));

	return 0;
}
arch_initcall_sync(lancer_init);

static __exit void lancer_exit(void) {
	/* Actually, this cannot be unloaded. Or loaded as a module..? */
} 
module_exit(lancer_exit);

MODULE_DESCRIPTION("Lancer expansion board driver");
MODULE_LICENSE("GPL");
