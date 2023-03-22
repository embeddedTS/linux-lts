/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __LINUX_MFD_TS_SUPERVISOR_H
#define __LINUX_MFD_TS_SUPERVISOR_H

struct ts_supervisor {
	struct i2c_client *client;
	struct regmap *regmap;
	struct platform_device *rstc_pdev;
	struct platform_device *adc_pdev;
    struct platform_device *temp_pdev;
};

/* I2C Register addresses */
#define SUPER_MODEL         0
#define SUPER_REV_INFO      1
#define SUPER_ADC_CHAN_ADV  2
#define SUPER_FEATURES0     3
#define SUPER_CMDS          8
#define SUPER_FLAGS         16
#define SUPER_INPUTS        24
#define SUPER_REBOOT_REASON 32
#define SUPER_SERIAL0       34
#define SUPER_SERIAL1       35
#define SUPER_SERIAL2       36
#define SUPER_SERIAL_CTRL   37
#define SUPER_ADC_BASE      128
#define SUPER_ADC_LAST      159
#define SUPER_TEMPERATURE   160

enum gen_flags_t {
    FLG_FORCE_USB_CON = (1 << 4),
    FLG_LED_DAT = (1 << 3),
    FLG_OVERRIDE_LED = (1 << 2),
    FLG_WAKE_EN = (1 << 1),
};

enum gen_inputs_t {
    INPUTS_USB_VBUS = (1 << 0),
};

enum super_features_t {
    SUPER_FEAT_SN = (1 << 2),
    SUPER_FEAT_FWUPD = (1 << 1),
    SUPER_FEAT_RSTC = (1 << 0),
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

enum super_cmds_t {
    I2C_REBOOT = (1 << 0),
    I2C_HALT = (1 << 1),
};

enum super_serial_ctrl_t {
    SUPER_SN_LOCKED = (1 << 0),
};

#endif