/*
    Wandboard board file used for BCM AR6MXCS. Copyright (C) 2013 Tapani Utriainen
        Author: James Martinez
	Based from original Wandboard board file Authors: Tapani Utriainen, Edward Lin


    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/phy.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_device.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <sound/wm8960.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/ahci_sata.h>
#include <mach/common.h>
#include <mach/devices-common.h>
#include <mach/gpio.h>
#include <mach/iomux-mx6dl.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-v3.h>
#include <mach/mx6.h>
#include <mach/hardware.h>
#include <mach/mxc_hdmi.h>
#include <mach/viv_gpu.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include "crm_regs.h"
#include "devices-imx6q.h"
#include "usb.h"



/* Syntactic sugar for pad configuration */
#define IMX6_SETUP_PAD(p) \
	if (cpu_is_mx6q()) \
		mxc_iomux_v3_setup_pad(MX6Q_PAD_##p); \
	else \
		mxc_iomux_v3_setup_pad(MX6DL_PAD_##p)

/* See arch/arm/plat-mxc/include/mach/iomux-mx6dl.h for definitions */
static struct clk *clko;

/****************************************************************************
 *
 * DMA controller init
 *
 ****************************************************************************/

static __init void wand_init_dma(void) {
        imx6q_add_dma();
}

static int caam_enabled;

/****************************************************************************
 *
 * SD init
 *
 * SD4 is eMMC

 * SD3 is micro sd
 *
 ****************************************************************************/

static const struct esdhc_platform_data wand_sd_data[2] = {
 {
        .always_present = 1,
        .wp_gpio = -EINVAL,
        .keep_power_at_suspend = 1,
        .support_8bit = 1,
        .delay_line = 0,
        .cd_type = ESDHC_CD_PERMANENT,
},
	 {
        .cd_gpio		= IMX_GPIO_NR(7, 0),
        .wp_gpio = -EINVAL,
        .keep_power_at_suspend = 1,
        .support_8bit = 0,
        .delay_line = 0,
        .cd_type = ESDHC_CD_CONTROLLER,
}
};

/* ------------------------------------------------------------------------ */

static void wand_init_sd(void) {
	int i;

	IMX6_SETUP_PAD( SD3_CLK__USDHC3_CLK_50MHZ );
	IMX6_SETUP_PAD( SD3_CMD__USDHC3_CMD_50MHZ );
	IMX6_SETUP_PAD( SD3_DAT0__USDHC3_DAT0_50MHZ );
	IMX6_SETUP_PAD( SD3_DAT1__USDHC3_DAT1_50MHZ );
	IMX6_SETUP_PAD( SD3_DAT2__USDHC3_DAT2_50MHZ );
	IMX6_SETUP_PAD( SD3_DAT3__USDHC3_DAT3_50MHZ );

    IMX6_SETUP_PAD( SD4_CLK__USDHC4_CLK_50MHZ );
	IMX6_SETUP_PAD( SD4_CMD__USDHC4_CMD_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT0__USDHC4_DAT0_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT1__USDHC4_DAT1_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT2__USDHC4_DAT2_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT3__USDHC4_DAT3_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT4__USDHC4_DAT4_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT5__USDHC4_DAT5_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT6__USDHC4_DAT6_50MHZ );
	IMX6_SETUP_PAD( SD4_DAT7__USDHC4_DAT7_50MHZ );

	/* Card Detect for SD3*/
	IMX6_SETUP_PAD( SD3_DAT5__GPIO_7_0 );

	/* Add mmc devices in reverse order, so mmc0 always is boot sd (SD3) */
	imx6q_add_sdhci_usdhc_imx(2, &wand_sd_data[0]);
	imx6q_add_sdhci_usdhc_imx(3, &wand_sd_data[1]);
}


/****************************************************************************
 *
 * I2C
 *
 ****************************************************************************/
static struct mxc_audio_platform_data wm8960_data;

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data = &wm8960_data,
	},
};


static struct pca953x_platform_data gpio_ext1 = {
    .gpio_base =  -1,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {

};


static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("s35390a", 0x30),
		.irq  = gpio_to_irq(IMX_GPIO_NR(6, 7)),
	},
	{
		I2C_BOARD_INFO("pca9633", 0x62),
	},
	{
		I2C_BOARD_INFO("pca9539", 0x74),
		//.irq  = gpio_to_irq(IMX_GPIO_NR(2, 0)),
		.platform_data = &gpio_ext1,
	},
};

static struct imxi2c_platform_data wand_i2c_data[] = {
	{ .bitrate	= 100000, },
	{ .bitrate	= 100000, },
	{ .bitrate	= 100000, },
};

/* ------------------------------------------------------------------------ */

static void __init wand_init_i2c(void) {
        int i;

    IMX6_SETUP_PAD( CSI0_DAT9__I2C1_SCL );
	IMX6_SETUP_PAD( CSI0_DAT8__I2C1_SDA );

	IMX6_SETUP_PAD( KEY_COL3__I2C2_SCL );
	IMX6_SETUP_PAD( KEY_ROW3__I2C2_SDA );

	IMX6_SETUP_PAD( GPIO_3__I2C3_SCL );
	IMX6_SETUP_PAD( GPIO_6__I2C3_SDA );

	for (i=0; i<3; i++) {
		imx6q_add_imx_i2c(i, &wand_i2c_data[i]);

        }


    i2c_register_board_info(0, mxc_i2c0_board_info,ARRAY_SIZE(mxc_i2c0_board_info));
    i2c_register_board_info(1, mxc_i2c1_board_info,ARRAY_SIZE(mxc_i2c1_board_info));
    i2c_register_board_info(2, mxc_i2c2_board_info,ARRAY_SIZE(mxc_i2c2_board_info));

}


/****************************************************************************
 *
 * Initialize debug console (UART1) & (UART4)
 *
 ****************************************************************************/

static __init void wand_init_uart(void) {
	/*PADS changed for BCM AR6MXCS */
    IMX6_SETUP_PAD(SD3_DAT7__UART1_TXD);
    IMX6_SETUP_PAD(SD3_DAT6__UART1_RXD);
    IMX6_SETUP_PAD(EIM_D19__UART1_CTS );
    IMX6_SETUP_PAD(EIM_D20__UART1_RTS );
	imx6q_add_imx_uart(0, NULL);
	IMX6_SETUP_PAD(KEY_COL0__UART4_TXD);
    IMX6_SETUP_PAD(KEY_ROW0__UART4_RXD);
	imx6q_add_imx_uart(3, NULL);
}


/****************************************************************************
 *
 * Initialize sound (SSI, ASRC, AUD3 channel and S/PDIF)
 *
 ****************************************************************************/
static struct imx_ssi_platform_data mx6_ar6mxcs_ssi1_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct platform_device mx6_ar6mxcs_audio_wm8960_device = {
	.name = "imx-wm8960",
};

static struct mxc_audio_platform_data wm8960_data;

static int wm8960_clk_enable(int enable)
{
	if (enable)
		clk_enable(clko);
	else
		clk_disable(clko);
	return 0;
}

static int mxc_wm8960_init(void)
{
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "clko2_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 24000000);
	pr_err("WM8960 mclk freq %d!\n", rate);
	clk_set_rate(clko, rate);
	wm8960_data.sysclk = rate;
	clk_enable(clko);

	return 0;
}

static struct mxc_audio_platform_data wm8960_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = -1,
	.mic_gpio = -1,
	.init = mxc_wm8960_init,
	.clock_enable = wm8960_clk_enable,
};

static int __init wand_init_audio(void)
{
  IMX6_SETUP_PAD(CSI0_MCLK__CCM_CLKO);
  IMX6_SETUP_PAD(CSI0_DAT4__AUDMUX_AUD3_TXC);
  IMX6_SETUP_PAD(CSI0_DAT5__AUDMUX_AUD3_TXD);
  IMX6_SETUP_PAD(CSI0_DAT6__AUDMUX_AUD3_TXFS);
  IMX6_SETUP_PAD(CSI0_DAT7__AUDMUX_AUD3_RXD);
  IMX6_SETUP_PAD(SD1_DAT0__GPIO_1_16);
  IMX6_SETUP_PAD(SD1_CLK__GPIO_1_20);
  IMX6_SETUP_PAD(ENET_CRS_DV__SPDIF_SPDIF_EXTCLK);
  IMX6_SETUP_PAD(GPIO_8__SPDIF_SRCLK);


    mxc_register_device(&mx6_ar6mxcs_audio_wm8960_device, &wm8960_data);
	imx6q_add_imx_ssi(1, &mx6_ar6mxcs_ssi1_pdata);

	mxc_wm8960_init();

	return 0;
}



/*****************************************************************************
 *
 * Init FEC and  PHY
 *
 *****************************************************************************/

static int wand_fec_phy_init(struct phy_device *phydev) {

	/* RX Data Pad Skew Register */
	phy_write(phydev, 0xd, 0x0002);
	phy_write(phydev, 0xe, 0x0005);
	phy_write(phydev, 0xd, 0xc002);
	phy_write(phydev, 0xe, 0x7777);

	/* TX Data Pad Skew Register */
	phy_write(phydev, 0xd, 0x0002);
	phy_write(phydev, 0xe, 0x0006);
	phy_write(phydev, 0xd, 0xc002);
	phy_write(phydev, 0xe, 0x7777);

	/* rx/tx data delay no changed, clock set max */
	phy_write(phydev, 0xd, 0x0002);
	phy_write(phydev, 0xe, 0x0008);
	phy_write(phydev, 0xd, 0xc002);
	phy_write(phydev, 0xe, 0x7fff);

	return 0;
}


/* ------------------------------------------------------------------------ */

static struct fec_platform_data wand_fec_data = {
	.init			= wand_fec_phy_init,
	.phy			= PHY_INTERFACE_MODE_RGMII,
	//.gpio_irq       = IMX_GPIO_NR(1, 28),
};

/* ------------------------------------------------------------------------ */

static __init void wand_init_ethernet(void) {
	IMX6_SETUP_PAD( ENET_MDIO__ENET_MDIO );
	IMX6_SETUP_PAD( ENET_MDC__ENET_MDC );
	IMX6_SETUP_PAD( ENET_REF_CLK__ENET_TX_CLK );
	IMX6_SETUP_PAD( RGMII_TXC__ENET_RGMII_TXC );
	IMX6_SETUP_PAD( RGMII_TD0__ENET_RGMII_TD0 );
	IMX6_SETUP_PAD( RGMII_TD1__ENET_RGMII_TD1 );
	IMX6_SETUP_PAD( RGMII_TD2__ENET_RGMII_TD2 );
	IMX6_SETUP_PAD( RGMII_TD3__ENET_RGMII_TD3 );
	IMX6_SETUP_PAD( RGMII_TX_CTL__ENET_RGMII_TX_CTL );
	IMX6_SETUP_PAD( RGMII_RXC__ENET_RGMII_RXC );
	IMX6_SETUP_PAD( RGMII_RD0__ENET_RGMII_RD0 );
	IMX6_SETUP_PAD( RGMII_RD1__ENET_RGMII_RD1 );
	IMX6_SETUP_PAD( RGMII_RD2__ENET_RGMII_RD2 );
	IMX6_SETUP_PAD( RGMII_RD3__ENET_RGMII_RD3 );
	IMX6_SETUP_PAD( RGMII_RX_CTL__ENET_RGMII_RX_CTL );
	IMX6_SETUP_PAD( ENET_TX_EN__GPIO_1_28 ); /* PHY_INT    */
	IMX6_SETUP_PAD( ENET_RXD0__GPIO_1_27 );  /* PHY_RST    */

		gpio_request(IMX_GPIO_NR(1, 27), "PHY_RST");
        gpio_direction_output(IMX_GPIO_NR(1, 27), 0);
        msleep(10);
        gpio_set_value(IMX_GPIO_NR(1, 27), 1);

	imx6_init_fec(wand_fec_data);
}


/****************************************************************************
 *
 * USB
 *
 ****************************************************************************/

static void imx6q_ar6mxcs_usbotg_vbus(bool on){}

static void imx6q_ar6mxcs_host1_vbus(bool on){}

/* ------------------------------------------------------------------------ */

static __init void wand_init_usb(void) {

       //IMX6_SETUP_PAD( GPIO_1__USBOTG_ID );


	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	mx6_set_otghost_vbus_func(imx6q_ar6mxcs_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_ar6mxcs_host1_vbus);
}


/****************************************************************************
 *
 * IPU
 *
 ****************************************************************************/

static struct imx_ipuv3_platform_data wand_ipu_data[] = {
	{
		.rev		= 4,
		.csi_clk[0]	= "ccm_clk0",
	}, {
		.rev		= 4,
		.csi_clk[0]	= "ccm_clk0",
	},
};

/* ------------------------------------------------------------------------ */

static __init void wand_init_ipu(void) {
	imx6q_add_ipuv3(0, &wand_ipu_data[0]);
	if (cpu_is_mx6q()) imx6q_add_ipuv3(1, &wand_ipu_data[1]);
}


/****************************************************************************
 *
 * HDMI
 *
 ****************************************************************************/


static struct ipuv3_fb_platform_data wand_hdmi_fb[] = {
	{ /* hdmi framebuffer */
		.disp_dev		= "hdmi",
		.interface_pix_fmt	= IPU_PIX_FMT_RGB24,
		.mode_str		= "1920x1080@60",
		.default_bpp		= 32,
		.int_clk		= false,
	}
};

/* ------------------------------------------------------------------------ */

static void wand_hdmi_init(int ipu_id, int disp_id) {
	if ((unsigned)ipu_id > 1) ipu_id = 0;
	if ((unsigned)disp_id > 1) disp_id = 0;

	mxc_iomux_set_gpr_register(3, 2, 2, 2*ipu_id + disp_id);
}

/* ------------------------------------------------------------------------ */

static struct fsl_mxc_hdmi_platform_data wand_hdmi_data = {
	.init = wand_hdmi_init,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};

/* ------------------------------------------------------------------------ */

static struct fsl_mxc_hdmi_core_platform_data wand_hdmi_core_data = {
	.ipu_id		= 0,
	.disp_id	= 1,
};

/* ------------------------------------------------------------------------ */

static const struct i2c_board_info wand_hdmi_i2c_info = {
	I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
};

/* ------------------------------------------------------------------------ */

static void wand_init_hdmi(void) {
    IMX6_SETUP_PAD(KEY_ROW2__HDMI_TX_CEC_LINE);
	i2c_register_board_info(1, &wand_hdmi_i2c_info, 1);
	imx6q_add_mxc_hdmi_core(&wand_hdmi_core_data);
	imx6q_add_mxc_hdmi(&wand_hdmi_data);
	imx6q_add_ipuv3fb(0, wand_hdmi_fb);

        /* Enable HDMI audio */
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);

}


/****************************************************************************
 *
 * LCD/LVDS/TTL
 *
 ****************************************************************************/

static struct fsl_mxc_lcd_platform_data wand_lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB666,
};

/* ------------------------------------------------------------------------ */

static struct fsl_mxc_ldb_platform_data wand_ldb_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SIN0,
	.sec_ipu_id = 0,
	.sec_disp_id = 1,
};

/* ------------------------------------------------------------------------ */

static struct ipuv3_fb_platform_data wand_lvds_fb[] = {
	{
		.disp_dev = "ldb",
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
		.mode_str = "LDB-WVGA",
		.default_bpp = 24,
		.int_clk = false,
        .late_init = false,
	},
};

/* ------------------------------------------------------------------------ */

static struct platform_pwm_backlight_data mx6_wand_pwm_backlight_data4 = {
	.pwm_id = 3,
	.max_brightness = 100,
	.dft_brightness = 100,
	.pwm_period_ns = 10000,
};

/* ------------------------------------------------------------------------ */

static void __init wand_init_lcd(void) {
	/* TTL */
	IMX6_SETUP_PAD( DI0_DISP_CLK__IPU1_DI0_DISP_CLK );
	IMX6_SETUP_PAD( DI0_PIN2__IPU1_DI0_PIN2 );		/* HSync */
	IMX6_SETUP_PAD( DI0_PIN3__IPU1_DI0_PIN3 );		/* VSync */
	IMX6_SETUP_PAD( DI0_PIN4__IPU1_DI0_PIN4 );		/* Contrast */
	IMX6_SETUP_PAD( DI0_PIN15__IPU1_DI0_PIN15 );		/* DISP0_DRDY */
	IMX6_SETUP_PAD( DISP0_DAT0__IPU1_DISP0_DAT_0 );
	IMX6_SETUP_PAD( DISP0_DAT1__IPU1_DISP0_DAT_1 );
	IMX6_SETUP_PAD( DISP0_DAT2__IPU1_DISP0_DAT_2 );
	IMX6_SETUP_PAD( DISP0_DAT3__IPU1_DISP0_DAT_3 );
	IMX6_SETUP_PAD( DISP0_DAT4__IPU1_DISP0_DAT_4 );
	IMX6_SETUP_PAD( DISP0_DAT5__IPU1_DISP0_DAT_5 );
	IMX6_SETUP_PAD( DISP0_DAT6__IPU1_DISP0_DAT_6 );
	IMX6_SETUP_PAD( DISP0_DAT7__IPU1_DISP0_DAT_7 );
	IMX6_SETUP_PAD( DISP0_DAT8__IPU1_DISP0_DAT_8 );
	IMX6_SETUP_PAD( DISP0_DAT9__IPU1_DISP0_DAT_9 );
	IMX6_SETUP_PAD( DISP0_DAT10__IPU1_DISP0_DAT_10 );
	IMX6_SETUP_PAD( DISP0_DAT11__IPU1_DISP0_DAT_11 );
	IMX6_SETUP_PAD( DISP0_DAT12__IPU1_DISP0_DAT_12 );
	IMX6_SETUP_PAD( DISP0_DAT13__IPU1_DISP0_DAT_13 );
	IMX6_SETUP_PAD( DISP0_DAT14__IPU1_DISP0_DAT_14 );
	IMX6_SETUP_PAD( DISP0_DAT15__IPU1_DISP0_DAT_15 );
	IMX6_SETUP_PAD( DISP0_DAT16__IPU1_DISP0_DAT_16 );
	IMX6_SETUP_PAD( DISP0_DAT17__IPU1_DISP0_DAT_17 );
	IMX6_SETUP_PAD( DISP0_DAT18__IPU1_DISP0_DAT_18 );
	IMX6_SETUP_PAD( DISP0_DAT19__IPU1_DISP0_DAT_19 );
	IMX6_SETUP_PAD( DISP0_DAT20__IPU1_DISP0_DAT_20 );
	IMX6_SETUP_PAD( DISP0_DAT21__IPU1_DISP0_DAT_21 );
	IMX6_SETUP_PAD( DISP0_DAT22__IPU1_DISP0_DAT_22 );
	IMX6_SETUP_PAD( DISP0_DAT23__IPU1_DISP0_DAT_23 );

	/* LVDS */
	IMX6_SETUP_PAD(KEY_COL1__GPIO_4_8);
	IMX6_SETUP_PAD( SD1_CMD__PWM4_PWMO);
	IMX6_SETUP_PAD(KEY_ROW1__GPIO_4_9);
	IMX6_SETUP_PAD(KEY_COL2__GPIO_4_10);


	IMX6_SETUP_PAD( LVDS0_CLK_P__LDB_LVDS0_CLK );
	IMX6_SETUP_PAD( LVDS0_TX0_P__LDB_LVDS0_TX0 );
	IMX6_SETUP_PAD( LVDS0_TX1_P__LDB_LVDS0_TX1 );
	IMX6_SETUP_PAD( LVDS0_TX2_P__LDB_LVDS0_TX2 );
	IMX6_SETUP_PAD( LVDS0_TX3_P__LDB_LVDS0_TX3 );

	gpio_request(IMX_GPIO_NR(4, 8), "L0_PWREN");
	gpio_export(IMX_GPIO_NR(4, 8), 1);
	gpio_direction_output(IMX_GPIO_NR(4, 8), 1);

    gpio_request(IMX_GPIO_NR(4, 9), "BL0_PWREN");
    gpio_export(IMX_GPIO_NR(4, 9), 1);
	gpio_direction_output(IMX_GPIO_NR(4, 9), 1);

    gpio_request(IMX_GPIO_NR(4, 10), "BL0EN");
    gpio_export(IMX_GPIO_NR(4, 10), 1);
    gpio_direction_output(IMX_GPIO_NR(4, 10), 1);

	imx6q_add_mxc_pwm(3);
    	imx6q_add_mxc_pwm_backlight(3, &mx6_wand_pwm_backlight_data4);

	imx6q_add_vdoa();

	imx6q_add_ldb(&wand_ldb_data);
	imx6q_add_lcdif(&wand_lcdif_data);

	imx6q_add_ipuv3fb(1, &wand_lvds_fb[0]);
}


/****************************************************************************
 *
 * Power and thermal management
 *
 ****************************************************************************/
 static const struct anatop_thermal_platform_data
	mx6q_ar6mxcs_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static void ar6mxcs_suspend_enter(void)
{
}

static void ar6mxcs_suspend_exit(void)
{
}

static const struct pm_platform_data mx6q_ar6mxcs_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = ar6mxcs_suspend_enter,
	.suspend_exit = ar6mxcs_suspend_exit,
};

static struct regulator_consumer_supply ar6mxcs_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data ar6mxcs_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(ar6mxcs_vmmc_consumers),
	.consumer_supplies = ar6mxcs_vmmc_consumers,
};

static struct fixed_voltage_config ar6mxcs_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &ar6mxcs_vmmc_init,
};

static struct platform_device ar6mxcs_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &ar6mxcs_vmmc_reg_config,
	},
};

static struct mxc_dvfs_platform_data ar6mxcs_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};


/* ------------------------------------------------------------------------ */

static __init void wand_init_pm(void) {
	imx6q_add_anatop_thermal_imx(1, &mx6q_ar6mxcs_anatop_thermal_data);
    imx6_add_armpmu();
	imx6q_add_pm_imx(0, &mx6q_ar6mxcs_pm_data);
	imx6q_add_dvfs_core(&ar6mxcs_dvfscore_data);
	imx6q_add_busfreq();
}


/****************************************************************************
 *
 * Expansion  GPIOs & WDT
 *
 ****************************************************************************/

 static struct gpio_led status_leds[] = {
          {
          .gpio = IMX_GPIO_NR(2, 1),
          .name = "LED1_PWR_BRD",
          .default_trigger = "heartbeat",
          .active_low = 1,
          },
            {
          .gpio = IMX_GPIO_NR(2, 2),
          .name = "LED2_PWR_BRD",
          .default_trigger = "mmc0",
          .active_low = 1,
          },
         {
          .gpio = IMX_GPIO_NR(6, 9),
          .name = "LED1_AR6MXCS",
          .default_trigger = "heartbeat",
          .active_low = 1,
          },
            {
          .gpio = IMX_GPIO_NR(6, 10),
          .name = "LED2_AR6MXCS",
          .default_trigger = "mmc0",
          .active_low = 1,
          },

};

static struct gpio_led_platform_data status_leds_data = {
	.num_leds	= ARRAY_SIZE(status_leds),
	.leds		= status_leds
};

static struct platform_device status_leds_dev = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &status_leds_data,
	},
};


static __init void wand_init_external_gpios(void) {
	IMX6_SETUP_PAD( GPIO_0__GPIO_1_0 ); /* SW_RST  */
	IMX6_SETUP_PAD( GPIO_4__GPIO_1_4); /* VER_B0 */
	IMX6_SETUP_PAD( GPIO_5__GPIO_1_5); /* VER_B1 */

	IMX6_SETUP_PAD( NANDF_D0__GPIO_2_0 ); // power board I2C INT
	IMX6_SETUP_PAD( NANDF_D1__GPIO_2_1 ); // Power board led 1 Red
	IMX6_SETUP_PAD( NANDF_D2__GPIO_2_2 ); // Power board led 2 Green
	IMX6_SETUP_PAD( NANDF_D3__GPIO_2_3 );
	IMX6_SETUP_PAD( NANDF_D4__GPIO_2_4 );
	IMX6_SETUP_PAD( NANDF_D5__GPIO_2_5 );
	IMX6_SETUP_PAD( NANDF_D6__GPIO_2_6 );
	IMX6_SETUP_PAD( NANDF_D7__GPIO_2_7 );

	IMX6_SETUP_PAD( NANDF_CLE__GPIO_6_7); /* RTC_INT */
	IMX6_SETUP_PAD( NANDF_ALE__GPIO_6_8 ); /* CAP_TCH_INT0 */
	IMX6_SETUP_PAD( NANDF_WP_B__GPIO_6_9); /* GPIO_LED1 */
	IMX6_SETUP_PAD( NANDF_RB0__GPIO_6_10 ); /* GPIO_LED2 */

platform_device_register(&status_leds_dev);
 imx6q_add_imx2_wdt(0, NULL);

}

/****************************************************************************
 *
 * Vivante GPU/VPU
 *
 ****************************************************************************/

static const __initconst struct imx_viv_gpu_data wand_gpu_data = {
	.phys_baseaddr = 0,
	.iobase_3d = GPU_3D_ARB_BASE_ADDR,
	.irq_3d = MXC_INT_GPU3D_IRQ,
	.iobase_2d = GPU_2D_ARB_BASE_ADDR,
	.irq_2d = MXC_INT_GPU2D_IRQ,
	.iobase_vg = OPENVG_ARB_BASE_ADDR,
	.irq_vg = MXC_INT_OPENVG_XAQ2,
};

static struct viv_gpu_platform_data wand_gpu_pdata = {
	.reserved_mem_size = 48 * SZ_1M,
};

struct wand_vout_mem_data {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct wand_vout_mem_data wand_vout_mem __initdata = {
	.res_msize = 48 * SZ_1M,
};

static __init void wand_init_gpu(void) {
	struct platform_device *voutdev;
	imx_add_viv_gpu(&wand_gpu_data, &wand_gpu_pdata);
	imx6q_add_vpu();
	voutdev = imx6q_add_v4l2_output(0);

	if (wand_vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    wand_vout_mem.res_mbase,
					    wand_vout_mem.res_mbase,
					    wand_vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
                                             DMA_MEMORY_EXCLUSIVE));
	}
}


/*****************************************************************************
 *
 * PCI Express
 *
 *****************************************************************************/

static const struct imx_pcie_platform_data wand_pcie_data = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= IMX_GPIO_NR(7, 11),
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= IMX_GPIO_NR(1, 9),
};

/* ------------------------------------------------------------------------ */

static void __init wand_init_pcie(void) {
	IMX6_SETUP_PAD( GPIO_9__GPIO_1_9); /* PCIE_DIS_B */
	IMX6_SETUP_PAD( GPIO_16__GPIO_7_11); /* PCIE_RST_B */
	imx6q_add_pcie(&wand_pcie_data);
}


/****************************************************************************
 *
 * CAAM - I.MX6 Cryptographic Acceleration
 *
 ****************************************************************************/

static int __init caam_setup(char *__unused)
{
        caam_enabled = 1;
        return 1;
}
early_param("caam", caam_setup);

static void __init wand_init_caam(void) {
	if (caam_enabled) {
		pr_info("CAAM loading\n");
		imx6q_add_imx_caam();
        }
}




/*****************************************************************************
 *
 * Init clocks and early boot console
 *
 *****************************************************************************/

extern void __iomem *twd_base;

static void __init wand_init_timer(void) {
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);
	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

/* ------------------------------------------------------------------------ */

static struct sys_timer wand_timer = {
	.init = wand_init_timer,
};

/* ------------------------------------------------------------------------ */

static void __init wand_reserve(void) {
	phys_addr_t phys;
	phys_addr_t total_mem = 0;
	struct meminfo *mi = &meminfo;
	int i;

	for (i=0; i<mi->nr_banks; i++)
		total_mem += mi->bank[i].size;

	if (wand_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(wand_gpu_pdata.reserved_mem_size, SZ_4K, total_mem);
		memblock_remove(phys, wand_gpu_pdata.reserved_mem_size);
		wand_gpu_pdata.reserved_mem_base = phys;
	}

	if (wand_vout_mem.res_msize) {
		phys = memblock_alloc_base(wand_vout_mem.res_msize, SZ_4K, total_mem);
		memblock_remove(phys, wand_vout_mem.res_msize);
		wand_vout_mem.res_mbase = phys;
	}
}


/*****************************************************************************
 *
 * BOARD INIT
 *
 *****************************************************************************/

static void __init wand_board_init(void) {
    wand_init_pm();
	wand_init_dma();
	wand_init_uart();
	wand_init_sd();
	wand_init_i2c();
	wand_init_audio();
	wand_init_ethernet();
	wand_init_usb();
	wand_init_ipu();
	wand_init_hdmi();
	wand_init_lcd();
	wand_init_external_gpios();
	wand_init_gpu();
	wand_init_caam();
	wand_init_pcie();

}

/* ------------------------------------------------------------------------ */

MACHINE_START(WANDBOARD, "Lancer-AR6MXCS")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= wand_board_init,
	.timer		= &wand_timer,
	.reserve        = wand_reserve,
MACHINE_END
