/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __LINUX_MFD_TS_WIZARD_H
#define __LINUX_MFD_TS_WIZARD_H

struct ts_wizard {
	struct i2c_client *client;
	struct regmap *regmap;
	struct platform_device *rstc_pdev;
	struct platform_device *adc_pdev;
    struct platform_device *temp_pdev;
};

/* I2C Register addresses */
#define WIZ_MODEL         0
#define WIZ_REV_INFO      1
#define WIZ_ADC_CHAN_ADV  2
#define WIZ_FEATURES0     3
#define WIZ_CMDS          8
#define WIZ_FLAGS         16
#define WIZ_INPUTS        24
#define WIZ_REBOOT_REASON 32
#define WIZ_SERIAL0       34
#define WIZ_SERIAL1       35
#define WIZ_SERIAL2       36
#define WIZ_SERIAL_CTRL   37
#define WIZ_SILO_BASE     64
#define WIZ_ADC_BASE      128
#define WIZ_ADC_LAST      159
#define WIZ_TEMPERATURE   160
#define WIZ_CURRENT       161
#define WIZ_IRQCHIP_BASE  512

enum gen_flags_t {
    FLG_FORCE_USB_CON = (1 << 4),
    FLG_LED_DAT = (1 << 3),
    FLG_OVERRIDE_LED = (1 << 2),
    FLG_WAKE_EN = (1 << 1),
};

enum gen_inputs_t {
    INPUTS_USB_VBUS = (1 << 0),
};

enum wiz_features_t {
   	WIZ_FEAT_CT = BIT(6),        // Channel Table visible
	WIZ_FEAT_SILO = BIT(5),
	WIZ_FEAT_BOOT_MODE = BIT(4),
	WIZ_FEAT_RBTR = BIT(3),      // TBI on i.MX93
    WIZ_FEAT_SN = (1 << 2),
    WIZ_FEAT_FWUPD = (1 << 1),
    WIZ_FEAT_RSTC = (1 << 0),
};

enum reboot_reasons_t {
    REBOOT_REASON_POR = 0,
    REBOOT_REASON_CPU_WDT = 1,
    REBOOT_REASON_SOFTWARE_REBOOT = 2,
    REBOOT_REASON_BROWNOUT = 3,
    REBOOT_REASON_RTC_ALARM_REBOOT = 4,
    REBOOT_REASON_WAKE_FROM_PWR_CYCLE = 5,
    REBOOT_REASON_WAKE_FROM_WAKE_SIGNAL = 6,
    REBOOT_REASON_WAKE_FROM_RTC_ALARM = 7,
    REBOOT_REASON_WAKE_FROM_USB_VBUS = 8,
};

enum wiz_cmds_t {
   	I2C_CMD_RESERVED3 = BIT(3),
	I2C_CMD_RESERVED2 = BIT(2),
    I2C_REBOOT = (1 << 0),
    I2C_HALT = (1 << 1),
};

enum wiz_serial_ctrl_t {
    WIZ_SN_LOCKED = (1 << 0),
};

#endif