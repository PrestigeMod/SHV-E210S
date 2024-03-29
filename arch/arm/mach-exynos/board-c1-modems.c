/* linux/arch/arm/mach-xxxx/board-c1-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>

/* inlcude platform specific file */
#include <linux/platform_data/modem.h>
#include <mach/sec_modem.h>
#include <mach/gpio.h>
#include <mach/gpio-exynos4.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-mem.h>
#include <plat/regs-srom.h>
#include <mach/cpufreq.h>
#include <mach/dev.h>
#include <linux/cpufreq_pegasusq.h>

#ifdef CONFIG_USBHUB_USB3503
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/platform_data/usb3503.h>
#include <plat/usb-phy.h>
#endif
#include <plat/devs.h>
#include <plat/ehci.h>

#define SROM_CS0_BASE		0x04000000
#define SROM_WIDTH		0x01000000
#define SROM_NUM_ADDR_BITS	14

#define C1ATT_REV_0_7	9	/* rev0.7 == system_rev:9 */

/* For "bus width and wait control (BW)" register */
enum sromc_attr {
	SROMC_DATA_16   = 0x1,	/* 16-bit data bus	*/
	SROMC_BYTE_ADDR = 0x2,	/* Byte base address	*/
	SROMC_WAIT_EN   = 0x4,	/* Wait enabled		*/
	SROMC_BYTE_EN   = 0x8,	/* Byte access enabled	*/
	SROMC_MASK      = 0xF
};

/* DPRAM configuration */
struct sromc_cfg {
	enum sromc_attr attr;
	unsigned size;
	unsigned csn;		/* CSn #			*/
	unsigned addr;		/* Start address (physical)	*/
	unsigned end;		/* End address (physical)	*/
};

/* DPRAM access timing configuration */
struct sromc_access_cfg {
	u32 tacs;		/* Address set-up before CSn		*/
	u32 tcos;		/* Chip selection set-up before OEn	*/
	u32 tacc;		/* Access cycle				*/
	u32 tcoh;		/* Chip selection hold on OEn		*/
	u32 tcah;		/* Address holding time after CSn	*/
	u32 tacp;		/* Page mode access cycle at Page mode	*/
	u32 pmc;		/* Page Mode config			*/
};

#define RES_CP_ACTIVE_IRQ_ID	0
#define RES_DPRAM_MEM_ID	1
#define RES_DPRAM_IRQ_ID	2

/* For CMC221 IDPRAM (Internal DPRAM) */
#define CMC_IDPRAM_SIZE		DPRAM_SIZE_16KB

/* For CMC221 SFR for IDPRAM */
#define CMC_INT2CP_REG		0x10	/* Interrupt to CP            */
#define CMC_INT2AP_REG		0x50
#define CMC_CLR_INT_REG		0x28	/* Clear Interrupt to AP      */
#define CMC_RESET_REG		0x3C
#define CMC_PUT_REG		0x40	/* AP->CP reg for hostbooting */
#define CMC_GET_REG		0x50	/* CP->AP reg for hostbooting */

/* Function prototypes */
static void config_dpram_port_gpio(int addr_bits);
static void init_sromc(void);
static void setup_sromc(unsigned csn, struct sromc_cfg *cfg,
		struct sromc_access_cfg *acc_cfg);
static void setup_dpram_speed(unsigned csn, struct sromc_access_cfg *acc_cfg);
static int __init init_modem(void);

static int host_port_enable(int port, int enable);

static struct sromc_cfg cmc_idpram_cfg = {
	.attr = SROMC_DATA_16,
	.size = CMC_IDPRAM_SIZE,
};

static struct sromc_access_cfg cmc_idpram_access_cfg[] = {
	[DPRAM_SPEED_LOW] = {
		/* for 33 MHz clock, 64 cycles */
		.tacs = 0x08 << 28,
		.tcos = 0x08 << 24,
		.tacc = 0x1F << 16,
		.tcoh = 0x08 << 12,
		.tcah = 0x08 << 8,
		.tacp = 0x00 << 4,
		.pmc  = 0x00 << 0,
	},
	[DPRAM_SPEED_MID] = {
		/* for 66 MHz clock, 32 cycles */
		.tacs = 0x01 << 28,
		.tcos = 0x01 << 24,
		.tacc = 0x1B << 16,
		.tcoh = 0x01 << 12,
		.tcah = 0x01 << 8,
		.tacp = 0x00 << 4,
		.pmc  = 0x00 << 0,
	},
	[DPRAM_SPEED_HIGH] = {
		/* for 133 MHz clock, 16 cycles */
		.tacs = 0x01 << 28,
		.tcos = 0x01 << 24,
		.tacc = 0x0B << 16,
		.tcoh = 0x01 << 12,
		.tcah = 0x01 << 8,
		.tacp = 0x00 << 4,
		.pmc  = 0x00 << 0,
	},
};

struct cmc221_idpram_sfr {
	u16 __iomem *int2cp;
	u16 __iomem *int2ap;
	u16 __iomem *clr_int2ap;
	u16 __iomem *reset;
	u16 __iomem *msg2cp;
	u16 __iomem *msg2ap;
};

static u8 *cmc_sfr_base;
static struct cmc221_idpram_sfr cmc_sfr;

static void cmc_idpram_reset(void)
{
	iowrite16(1, cmc_sfr.reset);
}

static void cmc_idpram_clr_intr(void)
{
	iowrite16(0xFFFF, cmc_sfr.clr_int2ap);
	iowrite16(0, cmc_sfr.int2ap);
}

static u16 cmc_idpram_recv_intr(void)
{
	return ioread16(cmc_sfr.int2ap);
}

static void cmc_idpram_send_intr(u16 irq_mask)
{
	iowrite16(irq_mask, cmc_sfr.int2cp);
}

static u16 cmc_idpram_recv_msg(void)
{
	return ioread16(cmc_sfr.msg2ap);
}

static void cmc_idpram_send_msg(u16 msg)
{
	iowrite16(msg, cmc_sfr.msg2cp);
}

static int cmc_idpram_wakeup(void);

static void cmc_idpram_sleep(void);

static void cmc_idpram_setup_speed(enum dpram_speed speed)
{
	setup_dpram_speed(cmc_idpram_cfg.csn, &cmc_idpram_access_cfg[speed]);
}

static struct modemlink_dpram_control cmc_idpram_ctrl = {
	.reset = cmc_idpram_reset,
	.clear_intr = cmc_idpram_clr_intr,
	.recv_intr = cmc_idpram_recv_intr,
	.send_intr = cmc_idpram_send_intr,
	.recv_msg = cmc_idpram_recv_msg,
	.send_msg = cmc_idpram_send_msg,

	.wakeup = cmc_idpram_wakeup,
	.sleep = cmc_idpram_sleep,

	.setup_speed = cmc_idpram_setup_speed,

	.dp_type = CP_IDPRAM,

	.dpram_irq_flags = (IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING),
};

/*
** UMTS target platform data
*/
static struct modem_io_t umts_io_devices[] = {
	[0] = {
		.name = "umts_boot0",
		.id = 0,
		.format = IPC_BOOT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[1] = {
		.name = "umts_ipc0",
		.id = 235,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[2] = {
		.name = "umts_rfs0",
		.id = 245,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[3] = {
		.name = "umts_multipdp",
		.id = 0,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.links = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB),
		.tx_link = LINKDEV_DPRAM,
	},
	[4] = {
		.name = "rmnet0",
		.id = 10,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB),
		.tx_link = LINKDEV_DPRAM,
	},
	[5] = {
		.name = "rmnet1",
		.id = 11,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB),
		.tx_link = LINKDEV_DPRAM,
	},
	[6] = {
		.name = "rmnet2",
		.id = 12,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB),
		.tx_link = LINKDEV_DPRAM,
	},
	[7] = {
		.name = "rmnet3",
		.id = 13,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.links = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB),
		.tx_link = LINKDEV_DPRAM,
	},
	[8] = {
		.name = "umts_csd",	/* CS Video Telephony */
		.id = 1,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[9] = {
		.name = "umts_router",	/* AT Iface & Dial-up */
		.id = 25,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[10] = {
		.name = "umts_dm0",	/* DM Port */
		.id = 28,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[11] = {
		.name = "umts_loopback_ap2cp",
		.id = 30,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[12] = {
		.name = "umts_loopback_cp2ap",
		.id = 31,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[13] = {
		.name = "umts_ramdump0",
		.id = 0,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[14] = {
		.name = "umts_log",
		.id = 0,
		.format = IPC_RAMDUMP,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_DPRAM),
	},
	[15] = {
		.name = "lte_ipc0",
		.id = 235,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.links = LINKTYPE(LINKDEV_USB),
	},
};

static int exynos_frequency_lock(struct device *dev);
static int exynos_frequency_unlock(struct device *dev);

static struct modemlink_pm_data umts_link_pm_data = {
	.name = "umts_link_pm",

	.gpio_link_enable    = 0,
	.gpio_link_active    = GPIO_ACTIVE_STATE,
	.gpio_link_hostwake  = GPIO_IPC_HOST_WAKEUP,
	.gpio_link_slavewake = GPIO_IPC_SLAVE_WAKEUP,

	.port_enable = host_port_enable,
/*
	.link_reconnect = umts_link_reconnect,
*/
	.freqlock = ATOMIC_INIT(0),
	.freq_lock = exynos_frequency_lock,
	.freq_unlock = exynos_frequency_unlock,

	.autosuspend_delay_ms = 2000,

	.has_usbhub = true, /* change this in init_modem if C1-ATT >= rev0.7 */
};

bool modem_using_hub(void)
{
	return umts_link_pm_data.has_usbhub;
}

static struct modemlink_pm_link_activectl active_ctl;

static struct modem_data umts_modem_data = {
	.name = "cmc221",

	.gpio_cp_on = CP_CMC221_PMIC_PWRON,
	.gpio_cp_reset = CP_CMC221_CPU_RST,
	.gpio_phone_active = GPIO_LTE_ACTIVE,
#if defined(CONFIG_MACH_C1_KOR_SKT) || defined(CONFIG_MACH_C1_KOR_KT)
	.gpio_pda_active   = GPIO_PDA_ACTIVE,
#endif

	.gpio_dpram_int = GPIO_CMC_IDPRAM_INT_00,
	.gpio_dpram_status = GPIO_CMC_IDPRAM_STATUS,
	.gpio_dpram_wakeup = GPIO_CMC_IDPRAM_WAKEUP,

	.gpio_slave_wakeup = GPIO_IPC_SLAVE_WAKEUP,
	.gpio_host_active = GPIO_ACTIVE_STATE,
	.gpio_host_wakeup = GPIO_IPC_HOST_WAKEUP,
	.gpio_dynamic_switching = GPIO_AP2CMC_INT2,

	.modem_net = UMTS_NETWORK,
	.modem_type = SEC_CMC221,
	.link_types = LINKTYPE(LINKDEV_DPRAM) | LINKTYPE(LINKDEV_USB),
	.link_name = "cmc221_idpram",
	.dpram_ctl = &cmc_idpram_ctrl,

	.num_iodevs = ARRAY_SIZE(umts_io_devices),
	.iodevs = umts_io_devices,

	.link_pm_data = &umts_link_pm_data,

	.ipc_version = SIPC_VER_50,
	.use_mif_log = true,
};

static struct resource umts_modem_res[] = {
	[RES_CP_ACTIVE_IRQ_ID] = {
		.name = "cp_active_irq",
		.start = LTE_ACTIVE_IRQ,
		.end = LTE_ACTIVE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	[RES_DPRAM_MEM_ID] = {
		.name = "dpram_base",
		.start = SROM_CS0_BASE,
		.end = SROM_CS0_BASE + (CMC_IDPRAM_SIZE - 1),
		.flags = IORESOURCE_MEM,
	},
	[RES_DPRAM_IRQ_ID] = {
		.name = "dpram_irq",
		.start = CMC_IDPRAM_INT_IRQ_00,
		.end = CMC_IDPRAM_INT_IRQ_00,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device umts_modem = {
	.name = "mif_sipc5",
	.id = 1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data,
	},
};

void set_slave_wake(void)
{
	int hostwake = umts_link_pm_data.gpio_link_hostwake;
	int slavewake = umts_link_pm_data.gpio_link_slavewake;

	if (gpio_get_value(slavewake)) {
		gpio_direction_output(slavewake, 0);
		mif_info("> S-WUP 0\n");
		mdelay(10);
	}
	gpio_direction_output(slavewake, 1);
	mif_info("> S-WUP 1\n");
}

void set_hsic_lpa_states(int states)
{
	int val = gpio_get_value(umts_modem_data.gpio_cp_reset);
	struct modemlink_pm_data *pm_data = &umts_link_pm_data;

	mif_trace("\n");

	if (val) {
		switch (states) {
		case STATE_HSIC_LPA_ENTER:
			mif_info("lpa_enter\n");
			/* gpio_link_active == gpio_host_active in C1 */
			gpio_set_value(umts_modem_data.gpio_host_active, 0);
			mif_info("> H-ACT %d\n", 0);
			if (pm_data->hub_standby && pm_data->hub_pm_data)
				pm_data->hub_standby(pm_data->hub_pm_data);
			break;
		case STATE_HSIC_LPA_WAKE:
			mif_info("lpa_wake\n");
			gpio_set_value(umts_modem_data.gpio_host_active, 1);
			mif_info("> H-ACT %d\n", 1);
			break;
		case STATE_HSIC_LPA_PHY_INIT:
			mif_info("lpa_phy_init\n");
			if (!modem_using_hub() && active_ctl.gpio_initialized)
				set_slave_wake();
			break;
		}
	}
}

int get_cp_active_state(void)
{
	return gpio_get_value(umts_modem_data.gpio_phone_active);
}

static int cmc_idpram_wakeup(void)
{
	int cnt = 0;

	gpio_set_value(umts_modem_data.gpio_dpram_wakeup, 1);

	while (!gpio_get_value(umts_modem_data.gpio_dpram_status)) {
		if (cnt++ > 10) {
			mif_err("ERR: gpio_dpram_status == 0\n");
			return -EAGAIN;
		}

		if (in_interrupt())
			mdelay(1);
		else
			msleep_interruptible(1);
	}

	return 0;
}

static void cmc_idpram_sleep(void)
{
	gpio_set_value(umts_modem_data.gpio_dpram_wakeup, 0);
}

/* Set dynamic environment for a modem */
static void setup_umts_modem_env(void)
{
	/* Config DPRAM control structure */
	cmc_idpram_cfg.csn  = 0;
	cmc_idpram_cfg.addr = SROM_CS0_BASE + (SROM_WIDTH * cmc_idpram_cfg.csn);
	cmc_idpram_cfg.end  = cmc_idpram_cfg.addr + cmc_idpram_cfg.size - 1;

	umts_modem_res[RES_DPRAM_MEM_ID].start = cmc_idpram_cfg.addr;
	umts_modem_res[RES_DPRAM_MEM_ID].end = cmc_idpram_cfg.end;

	umts_modem_data.gpio_dpram_int = GPIO_CMC_IDPRAM_INT_00;
}

static void config_umts_modem_gpio(void)
{
	int err;
	unsigned gpio_cp_on = umts_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst = umts_modem_data.gpio_cp_reset;
	unsigned gpio_pda_active = umts_modem_data.gpio_pda_active;
	unsigned gpio_phone_active = umts_modem_data.gpio_phone_active;
	unsigned gpio_active_state = umts_modem_data.gpio_host_active;
	unsigned gpio_host_wakeup = umts_modem_data.gpio_host_wakeup;
	unsigned gpio_slave_wakeup = umts_modem_data.gpio_slave_wakeup;
	unsigned gpio_dpram_int = umts_modem_data.gpio_dpram_int;
	unsigned gpio_dpram_status = umts_modem_data.gpio_dpram_status;
	unsigned gpio_dpram_wakeup = umts_modem_data.gpio_dpram_wakeup;
	unsigned gpio_dynamic_switching =
			umts_modem_data.gpio_dynamic_switching;

	if (gpio_cp_on) {
		err = gpio_request(gpio_cp_on, "CMC_ON");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n", "CMC_ON");
		} else {
			gpio_direction_output(gpio_cp_on, 0);
			s3c_gpio_setpull(gpio_cp_on, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_cp_rst) {
		err = gpio_request(gpio_cp_rst, "CMC_RST");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n", "CMC_RST");
		} else {
			gpio_direction_output(gpio_cp_rst, 0);
			s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_pda_active) {
		err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n", "PDA_ACTIVE");
		} else {
			gpio_direction_output(gpio_pda_active, 0);
			s3c_gpio_setpull(gpio_pda_active, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_phone_active) {
		err = gpio_request(gpio_phone_active, "CMC_ACTIVE");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n", "CMC_ACTIVE");
		} else {
			/* Configure as a wake-up source */
			gpio_direction_input(gpio_phone_active);
			s3c_gpio_setpull(gpio_phone_active, S3C_GPIO_PULL_DOWN);
			s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
		}
	}

	if (gpio_active_state) {
		err = gpio_request(gpio_active_state, "CMC_ACTIVE_STATE");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
				"CMC_ACTIVE_STATE");
		} else {
			gpio_direction_output(gpio_active_state, 0);
			s3c_gpio_setpull(gpio_active_state, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_slave_wakeup) {
		err = gpio_request(gpio_slave_wakeup, "CMC_SLAVE_WAKEUP");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
				"CMC_SLAVE_WAKEUP");
		} else {
			gpio_direction_output(gpio_slave_wakeup, 0);
			s3c_gpio_setpull(gpio_slave_wakeup, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_host_wakeup) {
		err = gpio_request(gpio_host_wakeup, "CMC_HOST_WAKEUP");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
				"CMC_HOST_WAKEUP");
		} else {
			/* Configure as a wake-up source */
			gpio_direction_input(gpio_host_wakeup);
			s3c_gpio_setpull(gpio_host_wakeup, S3C_GPIO_PULL_DOWN);
			s3c_gpio_cfgpin(gpio_host_wakeup, S3C_GPIO_SFN(0xF));
		}
	}

	if (gpio_dpram_int) {
		err = gpio_request(gpio_dpram_int, "CMC_DPRAM_INT");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
				"CMC_DPRAM_INT");
		} else {
			/* Configure as a wake-up source */
			gpio_direction_input(gpio_dpram_int);
			s3c_gpio_setpull(gpio_dpram_int, S3C_GPIO_PULL_NONE);
			s3c_gpio_cfgpin(gpio_dpram_int, S3C_GPIO_SFN(0xF));
		}
	}

	if (gpio_dpram_status) {
		err = gpio_request(gpio_dpram_status, "CMC_DPRAM_STATUS");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
				"CMC_DPRAM_STATUS");
		} else {
			gpio_direction_input(gpio_dpram_status);
			s3c_gpio_setpull(gpio_dpram_status, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_dpram_wakeup) {
		err = gpio_request(gpio_dpram_wakeup, "CMC_DPRAM_WAKEUP");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
				"CMC_DPRAM_WAKEUP");
		} else {
			gpio_direction_output(gpio_dpram_wakeup, 1);
			s3c_gpio_setpull(gpio_dpram_wakeup, S3C_GPIO_PULL_NONE);
		}
	}

	if (gpio_dynamic_switching) {
		err = gpio_request(gpio_dynamic_switching, "DYNAMIC_SWITCHING");
		if (err) {
			mif_err("ERR: fail to request gpio %s\n",
					"DYNAMIC_SWITCHING\n");
		} else {
			gpio_direction_input(gpio_dynamic_switching);
			s3c_gpio_setpull(gpio_dynamic_switching,
					S3C_GPIO_PULL_DOWN);
		}
	}

	active_ctl.gpio_initialized = 1;
	mif_info("done\n");
}

static u8 *cmc_idpram_remap_sfr_region(struct sromc_cfg *cfg)
{
	int dp_addr = cfg->addr;
	int dp_size = cfg->size;
	u8 __iomem *sfr_base;

	/* Remap DPRAM SFR region */
	dp_addr = cfg->addr + cfg->size;
	dp_size = cfg->size;

	sfr_base = (u8 __iomem *)ioremap_nocache(dp_addr, dp_size);
	if (!sfr_base) {
		mif_err("ERR: ioremap_nocache fail\n");
		return NULL;
	}

	cmc_sfr_base = sfr_base;

	cmc_sfr.int2cp = (u16 __iomem *)(sfr_base + CMC_INT2CP_REG);
	cmc_sfr.int2ap = (u16 __iomem *)(sfr_base + CMC_INT2AP_REG);
	cmc_sfr.clr_int2ap = (u16 __iomem *)(sfr_base + CMC_CLR_INT_REG);
	cmc_sfr.reset = (u16 __iomem *)(sfr_base + CMC_RESET_REG);
	cmc_sfr.msg2cp = (u16 __iomem *)(sfr_base + CMC_PUT_REG);
	cmc_sfr.msg2ap = (u16 __iomem *)(sfr_base + CMC_GET_REG);

	return sfr_base;
}

/**
 *	DPRAM GPIO settings
 *
 *	SROM_NUM_ADDR_BITS value indicate the address line number or
 *	the mux/demux dpram type. if you want to set mux mode, define the
 *	SROM_NUM_ADDR_BITS to zero.
 *
 *	for CMC22x
 *	CMC22x has 16KB + a SFR register address.
 *	It used 14 bits (13bits for 16KB word address and 1 bit for SFR
 *	register)
 */
static void config_dpram_port_gpio(int addr_bits)
{
	mif_info("address line = %d bits\n", addr_bits);

	/*
	** Config DPRAM address/data GPIO pins
	*/

	/* Set GPIO for address bus (13 ~ 14 bits) */
	switch (addr_bits) {
	case 0:
		break;

	case 13 ... 14:
		s3c_gpio_cfgrange_nopull(GPIO_SROM_ADDR_BUS_LOW,
			EXYNOS4_GPIO_Y3_NR, S3C_GPIO_SFN(2));
		s3c_gpio_cfgrange_nopull(GPIO_SROM_ADDR_BUS_HIGH,
			(addr_bits - EXYNOS4_GPIO_Y3_NR), S3C_GPIO_SFN(2));
		break;

	default:
		mif_err("ERR: invalid addr_bits!!!\n");
		return;
	}

	/* Set GPIO for data bus (16 bits) */
	s3c_gpio_cfgrange_nopull(GPIO_SROM_DATA_BUS_LOW, 8, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(GPIO_SROM_DATA_BUS_HIGH, 8, S3C_GPIO_SFN(2));

	/* Setup SROMC CSn pins */
	s3c_gpio_cfgpin(GPIO_DPRAM_CSN0, S3C_GPIO_SFN(2));

	/* Config OEn, WEn */
	s3c_gpio_cfgpin(GPIO_DPRAM_REN, S3C_GPIO_SFN(2));
	s3c_gpio_cfgpin(GPIO_DPRAM_WEN, S3C_GPIO_SFN(2));
}

static void init_sromc(void)
{
	struct clk *clk = NULL;

	/* SROMC clk enable */
	clk = clk_get(NULL, "sromc");
	if (!clk) {
		mif_err("ERR: SROMC clock gate fail\n");
		return;
	}
	clk_enable(clk);
}

static void setup_sromc
(
	unsigned csn,
	struct sromc_cfg *cfg,
	struct sromc_access_cfg *acc_cfg
)
{
	unsigned bw = 0;	/* Bus width and wait control	*/
	unsigned bc = 0;	/* Vank control			*/
	void __iomem *bank_sfr = S5P_SROM_BC0 + (4 * csn);

	mif_err("SROMC settings for CS%d...\n", csn);

	bw = __raw_readl(S5P_SROM_BW);
	bc = __raw_readl(bank_sfr);
	mif_err("Old SROMC settings = BW(0x%08X) BC%d(0x%08X)\n", bw, csn, bc);

	/* Set the BW control field for the CSn */
	bw &= ~(SROMC_MASK << (csn << 2));
	bw |= (cfg->attr << (csn << 2));
	writel(bw, S5P_SROM_BW);

	/* Set SROMC memory access timing for the CSn */
	bc = acc_cfg->tacs | acc_cfg->tcos | acc_cfg->tacc |
	     acc_cfg->tcoh | acc_cfg->tcah | acc_cfg->tacp | acc_cfg->pmc;

	writel(bc, bank_sfr);

	/* Verify SROMC settings */
	bw = __raw_readl(S5P_SROM_BW);
	bc = __raw_readl(bank_sfr);
	mif_err("New SROMC settings = BW(0x%08X) BC%d(0x%08X)\n", bw, csn, bc);
}

static void setup_dpram_speed(unsigned csn, struct sromc_access_cfg *acc_cfg)
{
	void __iomem *bank_sfr = S5P_SROM_BC0 + (4 * csn);
	unsigned bc = 0;

	bc = __raw_readl(bank_sfr);
	mif_info("Old CS%d setting = 0x%08X\n", csn, bc);

	/* SROMC memory access timing setting */
	bc = acc_cfg->tacs | acc_cfg->tcos | acc_cfg->tacc |
	     acc_cfg->tcoh | acc_cfg->tcah | acc_cfg->tacp | acc_cfg->pmc;
	writel(bc, bank_sfr);

	bc = __raw_readl(bank_sfr);
	mif_info("New CS%d setting = 0x%08X\n", csn, bc);
}

static int __init init_modem(void)
{
	struct sromc_cfg *cfg = NULL;
	struct sromc_access_cfg *acc_cfg = NULL;

#ifdef CONFIG_MACH_C1_USA_ATT
	/* check C1-ATT rev >=0.7
	 * <=rev0.6: <EXYNOS>--hsic--<HUB>--usb--<CMC221S>
	 * >=rev0.7: <EXYNOS>--hsic--------------<CMC221D>
	 */
	if (system_rev >= C1ATT_REV_0_7)
		umts_link_pm_data.has_usbhub = false;
#endif

	mif_err("System Revision = %d\n", system_rev);

	setup_umts_modem_env();

	config_dpram_port_gpio(SROM_NUM_ADDR_BITS);

	config_umts_modem_gpio();

	init_sromc();

	cfg = &cmc_idpram_cfg;
	acc_cfg = &cmc_idpram_access_cfg[DPRAM_SPEED_LOW];

	setup_sromc(cfg->csn, cfg, acc_cfg);

	if (!cmc_idpram_remap_sfr_region(&cmc_idpram_cfg))
		return -1;

	platform_device_register(&umts_modem);

	return 0;
}
late_initcall(init_modem);
/*device_initcall(init_modem);*/

#ifdef CONFIG_EXYNOS4_CPUFREQ
static int exynos_frequency_lock(struct device *dev)
{
	unsigned int level, cpufreq = 600; /* 200 ~ 1400 */
	unsigned int busfreq = 400200; /* 100100 ~ 400200 */
	int ret = 0;
	struct device *busdev = dev_get("exynos-busfreq");

	if (atomic_read(&umts_link_pm_data.freqlock) == 0) {
		/* cpu frequency lock */
		ret = exynos_cpufreq_get_level(cpufreq * 1000, &level);
		if (ret < 0) {
			mif_err("ERR: exynos_cpufreq_get_level fail: %d\n",
					ret);
			goto exit;
		}

		ret = exynos_cpufreq_lock(DVFS_LOCK_ID_USB_IF, level);
		if (ret < 0) {
			mif_err("ERR: exynos_cpufreq_lock fail: %d\n", ret);
			goto exit;
		}

		/* bus frequncy lock */
		if (!busdev) {
			mif_err("ERR: busdev is not exist\n");
			ret = -ENODEV;
			goto exit;
		}

		ret = dev_lock(busdev, dev, busfreq);
		if (ret < 0) {
			mif_err("ERR: dev_lock error: %d\n", ret);
			goto exit;
		}

		/* lock minimum number of cpu cores */
		cpufreq_pegasusq_min_cpu_lock(2);

		atomic_set(&umts_link_pm_data.freqlock, 1);
		mif_debug("level=%d, cpufreq=%d MHz, busfreq=%06d\n",
				level, cpufreq, busfreq);
	}
exit:
	return ret;
}

static int exynos_frequency_unlock(struct device *dev)
{
	int ret = 0;
	struct device *busdev = dev_get("exynos-busfreq");

	if (atomic_read(&umts_link_pm_data.freqlock) == 1) {
		/* cpu frequency unlock */
		exynos_cpufreq_lock_free(DVFS_LOCK_ID_USB_IF);

		/* bus frequency unlock */
		ret = dev_unlock(busdev, dev);
		if (ret < 0) {
			mif_err("ERR: dev_unlock error: %d\n", ret);
			goto exit;
		}

		/* unlock minimum number of cpu cores */
		cpufreq_pegasusq_min_cpu_unlock();

		atomic_set(&umts_link_pm_data.freqlock, 0);
		mif_debug("success\n");
	}
exit:
	return ret;
}
#else
static int exynos_frequency_lock(void)
{
	return 0;
}

static int exynos_frequency_unlock(void)
{
	return 0;
}
#endif

static int (*usbhub_set_mode)(struct usb3503_hubctl *, int);
static struct usb3503_hubctl *usbhub_ctl;

void set_host_states(struct platform_device *pdev, int type)
{
	if (modem_using_hub())
		return;

	if (active_ctl.gpio_initialized) {
		mif_err("%s: > H-ACT %d\n", pdev->name, type);
		gpio_direction_output(umts_link_pm_data.gpio_link_active, type);
	} else {
		active_ctl.gpio_request_host_active = 1;
	}
}

static int usb3503_hub_handler(void (*set_mode)(void), void *ctl)
{
	if (!set_mode || !ctl)
		return -EINVAL;

	usbhub_set_mode = (int (*)(struct usb3503_hubctl *, int))set_mode;
	usbhub_ctl = (struct usb3503_hubctl *)ctl;

	mif_info("set_mode(%pF)\n", set_mode);

	return 0;
}

static int usb3503_hw_config(void)
{
	int err;

	err = gpio_request(GPIO_USB_HUB_RST, "HUB_RST");
	if (err) {
		mif_err("ERR: fail to request gpio %s\n", "HUB_RST");
	} else {
		gpio_direction_output(GPIO_USB_HUB_RST, 0);
		s3c_gpio_setpull(GPIO_USB_HUB_RST, S3C_GPIO_PULL_NONE);
	}
	s5p_gpio_set_drvstr(GPIO_USB_HUB_RST, S5P_GPIO_DRVSTR_LV1);
	/* need to check drvstr 1 or 2 */

	/* for USB3503 26Mhz Reference clock setting */
	err = gpio_request(GPIO_USB_HUB_INT, "HUB_INT");
	if (err) {
		mif_err("ERR: fail to request gpio %s\n", "HUB_INT");
	} else {
		gpio_direction_output(GPIO_USB_HUB_INT, 1);
		s3c_gpio_setpull(GPIO_USB_HUB_INT, S3C_GPIO_PULL_NONE);
	}

	return 0;
}

static int usb3503_reset_n(int val)
{
	gpio_set_value(GPIO_USB_HUB_RST, 0);

	/* hub off from cpuidle(LPA), skip the msleep schedule*/
	if (val) {
		msleep(20);
		mif_info("val = %d\n", gpio_get_value(GPIO_USB_HUB_RST));

		gpio_set_value(GPIO_USB_HUB_RST, !!val);

		mif_info("val = %d\n", gpio_get_value(GPIO_USB_HUB_RST));
		udelay(5); /* need it ?*/
	}
	return 0;
}

static struct usb3503_platform_data usb3503_pdata = {
	.initial_mode = USB3503_MODE_STANDBY,
	.reset_n = usb3503_reset_n,
	.register_hub_handler = usb3503_hub_handler,
	.port_enable = host_port_enable,
};

static struct i2c_board_info i2c_devs20_emul[] __initdata = {
	{
		I2C_BOARD_INFO(USB3503_I2C_NAME, 0x08),
		.platform_data = &usb3503_pdata,
	},
};

/* I2C20_EMUL */
static struct i2c_gpio_platform_data i2c20_platdata = {
	.sda_pin = GPIO_USB_HUB_SDA,
	.scl_pin = GPIO_USB_HUB_SCL,
	/*FIXME: need to timming tunning...  */
	.udelay	= 20,
};

static struct platform_device s3c_device_i2c20 = {
	.name = "i2c-gpio",
	.id = 20,
	.dev.platform_data = &i2c20_platdata,
};

static int __init init_usbhub(void)
{
	usb3503_hw_config();
	i2c_register_board_info(20, i2c_devs20_emul,
				ARRAY_SIZE(i2c_devs20_emul));

	platform_device_register(&s3c_device_i2c20);
	return 0;
}

device_initcall(init_usbhub);

static int host_port_enable(int port, int enable)
{
	int err;

	mif_info("port(%d) control(%d)\n", port, enable);

	if (!modem_using_hub())
		return 0;

	if (enable) {
		err = usbhub_set_mode(usbhub_ctl, USB3503_MODE_HUB);
		if (err < 0) {
			mif_err("ERR: hub on fail\n");
			goto exit;
		}
		err = s5p_ehci_port_control(&s5p_device_ehci, port, 1);
		if (err < 0) {
			mif_err("ERR: port(%d) enable fail\n", port);
			goto exit;
		}
	} else {
		err = usbhub_set_mode(usbhub_ctl, USB3503_MODE_STANDBY);
		if (err < 0) {
			mif_err("ERR: hub off fail\n");
			goto exit;
		}
		err = s5p_ehci_port_control(&s5p_device_ehci, port, 0);
		if (err < 0) {
			mif_err("ERR: port(%d) enable fail\n", port);
			goto exit;
		}
	}

	err = gpio_direction_output(umts_modem_data.gpio_host_active, enable);
	mif_info("active state err(%d), en(%d), level(%d)\n",
		err, enable, gpio_get_value(umts_modem_data.gpio_host_active));

exit:
	return err;
}
