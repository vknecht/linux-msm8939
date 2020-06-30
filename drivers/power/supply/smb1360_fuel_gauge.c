// SPDX-License-Identifier: GPL-2.0-or-later

#include <asm/unaligned.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/* Charger Registers */
#define CFG_BATT_CHG_REG		0x00
#define CHG_ITERM_MASK			GENMASK(2, 0)
#define CHG_ITERM_25MA			0x0
#define CHG_ITERM_200MA			0x7
#define RECHG_MV_MASK			GENMASK(6, 5)
#define RECHG_MV_SHIFT			5

#define CFG_BATT_CHG_ICL_REG		0x05
#define AC_INPUT_ICL_PIN_BIT		BIT(7)
#define AC_INPUT_PIN_HIGH_BIT		BIT(6)

#define CFG_GLITCH_FLT_REG		0x06
#define AICL_ENABLED_BIT		BIT(0)

#define CFG_CHG_MISC_REG		0x7
#define CHG_EN_BY_PIN_BIT		BIT(7)
#define CHG_EN_ACTIVE_LOW_BIT		BIT(6)
#define PRE_TO_FAST_REQ_CMD_BIT		BIT(5)
#define CHG_CURR_TERM_DIS_BIT		BIT(3)
#define CFG_AUTO_RECHG_DIS_BIT		BIT(2)
#define CFG_CHG_INHIBIT_EN_BIT		BIT(0)

#define CFG_STAT_CTRL_REG		0x09
#define CHG_STAT_IRQ_ONLY_BIT		BIT(4)
#define CHG_STAT_ACTIVE_HIGH_BIT	BIT(1)
#define CHG_STAT_DISABLE_BIT		BIT(0)

#define CFG_SFY_TIMER_CTRL_REG		0x0A
#define SAFETY_TIME_EN_BIT		BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
#define SAFETY_TIME_MINUTES_MASK	GENMASK(3, 2)

#define CFG_BATT_MISSING_REG		0x0D
#define BATT_MISSING_SRC_THERM_BIT	BIT(1)

#define IRQ_CFG_REG			0x0F
#define IRQ_BAT_HOT_COLD_HARD_BIT	BIT(7)
#define IRQ_BAT_HOT_COLD_SOFT_BIT	BIT(6)
#define IRQ_DCIN_UV_BIT			BIT(2)
#define IRQ_INTERNAL_TEMPERATURE_BIT	BIT(0)
#define IRQ_CFG_FULL_CONF		(IRQ_BAT_HOT_COLD_HARD_BIT \
					| IRQ_BAT_HOT_COLD_SOFT_BIT \
					| IRQ_DCIN_UV_BIT \
					| IRQ_INTERNAL_TEMPERATURE_BIT)

#define IRQ2_CFG_REG			0x10
#define IRQ2_SAFETY_TIMER_BIT		BIT(7)
#define IRQ2_CHG_ERR_BIT		BIT(6)
#define IRQ2_CHG_PHASE_CHANGE_BIT	BIT(4)
#define IRQ2_POWER_OK_BIT		BIT(2)
#define IRQ2_BATT_MISSING_BIT		BIT(1)
#define IRQ2_VBAT_LOW_BIT		BIT(0)
#define IRQ2_CFG_FULL_CONF		(IRQ2_SAFETY_TIMER_BIT \
					| IRQ2_CHG_ERR_BIT \
					| IRQ2_CHG_PHASE_CHANGE_BIT \
					| IRQ2_POWER_OK_BIT \
					| IRQ2_BATT_MISSING_BIT \
					| IRQ2_VBAT_LOW_BIT)

#define IRQ3_CFG_REG			0x11
#define IRQ3_SOC_CHANGE_BIT		BIT(4)
#define IRQ3_SOC_MIN_BIT		BIT(3)
#define IRQ3_SOC_MAX_BIT		BIT(2)
#define IRQ3_SOC_EMPTY_BIT		BIT(1)
#define IRQ3_SOC_FULL_BIT		BIT(0)
#define IRQ3_CFG_FULL_CONF		(IRQ3_SOC_CHANGE_BIT \
					| IRQ3_SOC_MIN_BIT \
					| IRQ3_SOC_MAX_BIT \
					| IRQ3_SOC_EMPTY_BIT \
					| IRQ3_SOC_FULL_BIT)

#define BATT_CHG_FLT_VTG_REG		0x15
#define VFLOAT_MASK			GENMASK(6, 0)

/* Command Registers */
#define CMD_I2C_REG			0x40
#define ALLOW_VOLATILE_BIT		BIT(6)

#define CMD_CHG_REG			0x42
#define CMD_CHG_EN			BIT(1)

/* Status Registers */
#define STATUS_3_REG			0x4B
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			GENMASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3

#define REVISION_CTRL_REG		0x4F
#define DEVICE_REV_MASK			GENMASK(3, 0)

/* IRQ Status Registers */
#define IRQ_A_REG			0x50
#define IRQ_A_HOT_HARD_BIT		BIT(6)
#define IRQ_A_COLD_HARD_BIT		BIT(4)
#define IRQ_A_HOT_SOFT_BIT		BIT(2)
#define IRQ_A_COLD_SOFT_BIT		BIT(0)

#define IRQ_B_REG			0x51
#define IRQ_B_BATT_TERMINAL_BIT		BIT(6)
#define IRQ_B_BATT_MISSING_BIT		BIT(4)

#define IRQ_C_REG			0x52
#define IRQ_C_CHG_TERM			BIT(0)

#define IRQ_D_REG			0x53

#define IRQ_E_REG			0x54
#define IRQ_E_USBIN_UV_BIT		BIT(0)

#define IRQ_F_REG			0x55
#define IRQ_G_REG			0x56
#define IRQ_H_REG			0x57
#define IRQ_I_REG			0x58

/* FG registers - IRQ config register */
#define SOC_DELTA_REG			0x20
#define VTG_MIN_REG			0x23
#define SOC_MAX_REG			0x24
#define SOC_MIN_REG			0x25
#define VTG_EMPTY_REG			0x26

/* FG SHADOW registers */
#define SHDW_FG_MSYS_SOC		0x61
#define SHDW_FG_CAPACITY		0x62
#define SHDW_FG_VTG_NOW			0x69
#define SHDW_FG_CURR_NOW		0x6B
#define SHDW_FG_BATT_TEMP		0x6D

/* Constants */
#define MAX_8_BITS			255

#define SMB1360_REV_1			0x01

#define MIN_FLOAT_MV			3460
#define MAX_FLOAT_MV			4730
#define VFLOAT_STEP_MV			10

#define MIN_RECHG_MV			50
#define MAX_RECHG_MV			300

#define IRQ_LATCHED_MASK		0x02
#define IRQ_STATUS_MASK			0x01
#define BATT_ID_LATCHED_MASK		0x08
#define BATT_ID_STATUS_MASK		0x07
#define BITS_PER_IRQ			2

static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

static int is_between(int value, int left, int right)
{
	if (left >= right && left >= value && value >= right)
		return 1;
	if (left <= right && left <= value && value <= right)
		return 1;

	return 0;
}

struct smb1360_battery {
	struct i2c_client	*client;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*psy;
	struct mutex		irq_complete;
	struct mutex		charging_disable_lock;

	u8 revision;
	u8 irq_cfg_mask[3];

	int soc_max;
	int soc_min;
	int delta_soc;
	int voltage_min_mv;
	int voltage_empty_mv;
	int iterm_ma;
	int vfloat_mv;
	int safety_time;
	int resume_delta_mv;

	bool batt_present;
	bool usb_present;
	bool recharge_disabled;
	bool chg_inhibit_disabled;
	bool iterm_disabled;
	bool resume_completed;
	bool batt_cool;
	bool batt_warm;
	bool batt_cold;
	bool batt_hot;
	bool batt_full;
	bool irq_waiting;
	bool empty_soc;
};

struct smb1360_irq_info {
	const char *name;
	int (*smb_irq)(struct smb1360_battery *battery, u8 rt_stat);
};

struct smb1360_handler_info {
	u8 reg;
	u32 val;
	u32 prev_val;
	struct smb1360_irq_info irq_info[4];
};

static int smb1360_read_bytes(struct smb1360_battery *battery, int reg, u8 *val,
			      u8 bytes)
{
	s32 ret;

	ret = regmap_bulk_read(battery->regmap, reg, val, bytes);
	if (ret) {
		dev_err(&battery->client->dev,
			"failed to read from %d registry\n", reg);
		return ret;
	}

	return 0;
}

static int smb1360_write(struct smb1360_battery *battery, int reg, u8 val)
{
	s32 ret;

	ret = regmap_write(battery->regmap, reg, val);
	if (ret < 0) {
		dev_err(battery->dev,
			"failed to write %02x to %02x: %d\n", val, reg, ret);
		return ret;
	}

	return 0;
}

static int smb1360_masked_write(struct smb1360_battery *battery,
				int reg, u8 mask, u8 val)
{
	int ret;

	ret = regmap_update_bits(battery->regmap, reg, mask, val);
	if (ret < 0)
		dev_err(battery->dev, "failed to write %02x to %02x: %d\n",
			val, reg, ret);

	return ret;
}

static int smb1360_read_revision(struct smb1360_battery *battery, u8 *val)
{
	int ret;

	*val = 0;
	ret = smb1360_read_bytes(battery, REVISION_CTRL_REG, val, 1);
	*val &= DEVICE_REV_MASK;

	return ret;
}

static int smb1360_float_voltage_set(struct smb1360_battery *battery, int val)
{
	u8 temp;

	if ((val < MIN_FLOAT_MV) || (val > MAX_FLOAT_MV))
		return -EINVAL;

	temp = (val - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb1360_masked_write(battery, BATT_CHG_FLT_VTG_REG, VFLOAT_MASK, temp);
}

static int smb1360_recharge_threshold_set(struct smb1360_battery *battery,
					  int val)
{
	u8 temp;

	if ((val < MIN_RECHG_MV) || (val > MAX_RECHG_MV))
		return -EINVAL;

	temp = (val / 100) << RECHG_MV_SHIFT;

	return smb1360_masked_write(battery, CFG_BATT_CHG_REG, RECHG_MV_MASK, temp);
}

/* commented as unused atm, probably useful later
static int smb1360_charging_disable(struct smb1360_battery *battery,
				    bool disable)
{
	int ret;
	u8 val = disable ? 0 : CMD_CHG_EN;

	ret = smb1360_masked_write(battery, CMD_CHG_REG, CMD_CHG_EN, val);

	return ret;
}*/

static enum power_supply_property smb1360_battery_props[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int smb1360_get_prop_batt_present(struct smb1360_battery *battery)
{
	return battery->batt_present;
}

static int smb1360_get_prop_batt_status(struct smb1360_battery *battery)
{
	int ret;
	u8 reg;
	u32 chg_type;

	if (battery->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	ret = smb1360_read_bytes(battery, STATUS_3_REG, &reg, 1);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (reg & CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	return POWER_SUPPLY_STATUS_CHARGING;
}

static int smb1360_get_prop_charge_type(struct smb1360_battery *battery)
{
	int ret;
	u8 reg;
	u32 chg_type;

	ret = smb1360_read_bytes(battery, STATUS_3_REG, &reg, 1);
	if (ret)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	switch (chg_type) {
	case BATT_NOT_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	case BATT_FAST_CHG_VAL:
	case BATT_TAPER_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BATT_PRE_CHG_VAL:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
}

static int smb1360_get_prop_batt_health(struct smb1360_battery *battery)
{
	if (battery->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (battery->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static int smb1360_get_prop_batt_capacity(struct smb1360_battery *battery)
{
	int ret, soc = 0;
	u8 reg;

	if (battery->empty_soc)
		return 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_MSYS_SOC, &reg, 1);
	if (ret)
		return ret;

	soc = DIV_ROUND_CLOSEST((100 * reg), 255);
	soc = clamp(soc, 0, 100);

	return soc;
}

static int smb1360_get_prop_chg_full_design(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, fcc_mah = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_CAPACITY, reg, 2);
	if (ret)
		return ret;

	fcc_mah = get_unaligned_le16(reg);
	fcc_mah = fcc_mah * 1000;

	return fcc_mah;
}

static int smb1360_get_prop_batt_temp(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, temp = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_BATT_TEMP, reg, 2);
	if (ret)
		return ret;

	temp = get_unaligned_le16(reg);
	temp = div_u64(temp * 625, 10000UL); /* temperature in kelvin */
	temp = (temp - 273) * 10; /* temperature in decideg */

	return temp;
}

static int smb1360_get_prop_voltage_now(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, temp = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_VTG_NOW, reg, 2);
	if (ret)
		return ret;

	temp = get_unaligned_le16(reg);
	temp = div_u64(temp * 5000, 0x7FFF) * 1000;

	return temp;
}

static int smb1360_get_prop_current_now(struct smb1360_battery *battery)
{
	u8 reg[2];
	int ret, temp = 0;

	ret = smb1360_read_bytes(battery, SHDW_FG_CURR_NOW, reg, 2);
	if (ret)
		return ret;

	temp = (s16) get_unaligned_le16(reg);
	temp = div_s64(temp * 2500, 0x7FFF) * 1000;

	return temp;
}

static int smb1360_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct smb1360_battery *battery = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb1360_get_prop_batt_health(battery);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb1360_get_prop_batt_present(battery);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb1360_get_prop_batt_status(battery);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb1360_get_prop_charge_type(battery);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb1360_get_prop_batt_capacity(battery);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = smb1360_get_prop_chg_full_design(battery);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb1360_get_prop_voltage_now(battery);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb1360_get_prop_current_now(battery);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb1360_get_prop_batt_temp(battery);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void smb1360_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static int hot_hard_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_hot = !!rt_stat;
	return 0;
}

static int cold_hard_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_cold = !!rt_stat;
	return 0;
}

static int hot_soft_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_warm = !!rt_stat;
	return 0;
}

static int cold_soft_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_cool = !!rt_stat;
	return 0;
}

static int battery_missing_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_present = !rt_stat;
	return 0;
}

static int chg_term_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_full = !!rt_stat;
	return 0;
}

static int usbin_uv_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	bool usb_present = !rt_stat;
	battery->usb_present = usb_present;
	return 0;
}

static int chg_inhibit_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	battery->batt_full = !!rt_stat;
	return 0;
}

static int empty_soc_handler(struct smb1360_battery *battery, u8 rt_stat)
{
	if (rt_stat)
		battery->empty_soc = true;
	else
		battery->empty_soc = false;

	return 0;
}

static struct smb1360_handler_info handlers[] = {
	{ IRQ_A_REG, 0, 0, {
		{ .name = "cold_soft", .smb_irq = cold_soft_handler },
		{ .name = "hot_soft", .smb_irq = hot_soft_handler },
		{ .name = "cold_hard", .smb_irq = cold_hard_handler },
		{ .name = "hot_hard", .smb_irq = hot_hard_handler },
	}},
	{ IRQ_B_REG, 0, 0, {
		{ .name = "chg_hot", },
		{ .name = "vbat_low", },
		{ .name = "battery_missing", .smb_irq = battery_missing_handler },
		{ .name = "battery_missing", .smb_irq = battery_missing_handler },
	}},
	{ IRQ_C_REG, 0, 0, {
		{ .name = "chg_term", .smb_irq = chg_term_handler },
		{ .name = "taper", },
		{ .name = "recharge", },
		{ .name = "fast_chg", },
	}},
	{ IRQ_D_REG, 0, 0, {
		{ .name = "prechg_timeout", },
		{ .name = "safety_timeout", },
		{ .name = "aicl_done", },
		{ .name = "battery_ov" },
	}},
	{ IRQ_E_REG, 0, 0, {
		{ .name = "usbin_uv", .smb_irq = usbin_uv_handler },
		{ .name = "usbin_ov", },
		{ .name = "unused", },
		{ .name = "chg_inhibit", .smb_irq = chg_inhibit_handler },
	}},
	{ IRQ_F_REG, 0, 0, {
		{ .name = "power_ok", },
		{ .name = "unused", },
		{ .name = "otg_fail", },
		{ .name = "otg_oc", },
	}},
	{ IRQ_G_REG, 0, 0, {
		{ .name = "delta_soc", },
		{ .name = "chg_error", },
		{ .name = "wd_timeout", },
		{ .name = "unused", },
	}},
	{ IRQ_H_REG, 0, 0, {
		{ .name = "min_soc", },
		{ .name = "max_soc", },
		{ .name = "empty_soc", .smb_irq = empty_soc_handler},
		{ .name = "full_soc", },
	}},
	{ IRQ_I_REG, 0, 0, {
		{ .name = "fg_access_allowed", },
		{ .name = "fg_data_recovery", },
		{ .name = "batt_id_complete", },
	}},
};

static bool process_irq_handler(struct smb1360_battery *battery,
				struct smb1360_handler_info *handler)
{
	bool irq_active = false;
	u8 triggered, changed;
	u8 rt_stat, prev_rt_stat, irq_latched_mask, irq_status_mask;
	int j, rc, triggered_mask, status_mask;

	rc = regmap_read(battery->regmap, handler->reg, &handler->val);
	if (rc < 0) {
		dev_err(battery->dev,
			"couldn't read %d: %d\n", handler->reg, rc);
		return false;
	}

	for (j = 0; j < ARRAY_SIZE(handler->irq_info); j++) {
		if (handler->reg == IRQ_I_REG && j == 2) {
			irq_latched_mask = BATT_ID_LATCHED_MASK;
			irq_status_mask = BATT_ID_STATUS_MASK;
		} else {
			irq_latched_mask = IRQ_LATCHED_MASK;
			irq_status_mask = IRQ_STATUS_MASK;
		}

		triggered_mask = irq_latched_mask << (j * BITS_PER_IRQ);
		status_mask = irq_status_mask << (j * BITS_PER_IRQ);

		triggered = handler->val & triggered_mask;
		rt_stat = handler->val & status_mask;
		prev_rt_stat = handler->prev_val & status_mask;
		changed = prev_rt_stat ^ rt_stat;

		if (triggered || changed)
			irq_active = true;

		if (irq_active && handler->irq_info[j].smb_irq != NULL) {
			rc = handler->irq_info[j].smb_irq(battery, rt_stat);
			if (rc < 0)
				dev_err(battery->dev,
					"couldn't handle %d irq: %d\n", j, rc);
		}
	}
	handler->prev_val = handler->val;

	return irq_active;
}

static irqreturn_t smb1360_bat_irq(int irq, void *data)
{
	struct smb1360_battery *battery = data;
	bool result, handled = false;
	int i;

	mutex_lock(&battery->irq_complete);

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		result = process_irq_handler(battery, &handlers[i]);
		handled = handled ? handled : result;
	}

	if (handled)
		power_supply_changed(battery->psy);

	mutex_unlock(&battery->irq_complete);

	return IRQ_HANDLED;
}

static int determine_initial_status(struct smb1360_battery *battery)
{
	int ret;
	u8 val = 0;

	battery->batt_present = true;

	ret = smb1360_read_bytes(battery, IRQ_B_REG, &val, 1);
	if (ret < 0)
		return ret;

	if (val & IRQ_B_BATT_TERMINAL_BIT || val & IRQ_B_BATT_MISSING_BIT)
		battery->batt_present = false;

	ret = smb1360_read_bytes(battery, IRQ_C_REG, &val, 1);
	if (ret)
		return ret;

	if (val & IRQ_C_CHG_TERM)
		battery->batt_full = true;

	ret = smb1360_read_bytes(battery, IRQ_A_REG, &val, 1);
	if (ret < 0)
		return ret;

	if (val & IRQ_A_HOT_HARD_BIT)
		battery->batt_hot = true;
	if (val & IRQ_A_COLD_HARD_BIT)
		battery->batt_cold = true;
	if (val & IRQ_A_HOT_SOFT_BIT)
		battery->batt_warm = true;
	if (val & IRQ_A_COLD_SOFT_BIT)
		battery->batt_cool = true;

	ret = smb1360_read_bytes(battery, IRQ_E_REG, &val, 1);
	if (ret < 0)
		return ret;

	battery->usb_present = (val & IRQ_E_USBIN_UV_BIT) ? false : true;

	return 0;
}

static int smb1360_fg_config(struct smb1360_battery *battery)
{
	int ret, temp;
	u8 val = 0;

	if (battery->delta_soc != -EINVAL) {
		val = DIV_ROUND_UP(battery->delta_soc * MAX_8_BITS, 100);
		ret = smb1360_write(battery, SOC_DELTA_REG, val);
		if (ret)
			return ret;
	}

	if (battery->soc_min != -EINVAL && is_between(battery->soc_min, 0, 100)) {
		val = DIV_ROUND_UP(battery->soc_min * MAX_8_BITS, 100);
		ret = smb1360_write(battery, SOC_MIN_REG, val);
		if (ret)
			return ret;
	}

	if (battery->soc_max != -EINVAL && is_between(battery->soc_max, 0, 100)) {
		val = DIV_ROUND_UP(battery->soc_max * MAX_8_BITS, 100);
		ret = smb1360_write(battery, SOC_MAX_REG, val);
		if (ret)
			return ret;
	}

	if (battery->voltage_min_mv != -EINVAL) {
		temp = (battery->voltage_min_mv - 2500) * MAX_8_BITS;
		val = DIV_ROUND_UP(temp, 2500);
		ret = smb1360_write(battery, VTG_MIN_REG, val);
		if (ret)
			return ret;
	}

	if (battery->voltage_empty_mv != -EINVAL) {
		temp = (battery->voltage_empty_mv - 2500) * MAX_8_BITS;
		val = DIV_ROUND_UP(temp, 2500);
		ret = smb1360_write(battery, VTG_EMPTY_REG, val);
		if (ret)
			return ret;
	}

	return 0;
}

static int smb1360_hw_init(struct smb1360_battery *battery)
{
	int rc, i;
	u8 reg, mask;

	rc = smb1360_masked_write(battery, CMD_I2C_REG,
				  ALLOW_VOLATILE_BIT, ALLOW_VOLATILE_BIT);
	if (rc < 0) {
		dev_err(battery->dev, "couldn't configure for volatile: %d\n", rc);
		return rc;
	}

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast
	 */
	rc = smb1360_masked_write(battery, CFG_CHG_MISC_REG,
				  CHG_EN_BY_PIN_BIT
				  | CHG_EN_ACTIVE_LOW_BIT
				  | PRE_TO_FAST_REQ_CMD_BIT, 0);
	if (rc < 0)
		return rc;

	/* USB/AC pin settings */
	rc = smb1360_masked_write(battery, CFG_BATT_CHG_ICL_REG,
				  AC_INPUT_ICL_PIN_BIT | AC_INPUT_PIN_HIGH_BIT,
				  AC_INPUT_PIN_HIGH_BIT);
	if (rc < 0)
		return rc;

	/* AICL setting */
	rc = smb1360_masked_write(battery, CFG_GLITCH_FLT_REG,
				  AICL_ENABLED_BIT, AICL_ENABLED_BIT);
	if (rc < 0)
		return rc;

	/* set the float voltage */
	if (battery->vfloat_mv != -EINVAL) {
		rc = smb1360_float_voltage_set(battery, battery->vfloat_mv);
		if (rc < 0)
			return rc;
	}

	/* set iterm */
	if (battery->iterm_ma != -EINVAL) {
		if (battery->iterm_disabled) {
			dev_err(battery->dev, "iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (battery->iterm_ma < 25)
				reg = CHG_ITERM_25MA;
			else if (battery->iterm_ma > 200)
				reg = CHG_ITERM_200MA;
			else
				reg = DIV_ROUND_UP(battery->iterm_ma, 25) - 1;

			rc = smb1360_masked_write(battery, CFG_BATT_CHG_REG,
						  CHG_ITERM_MASK, reg);
			if (rc)
				return rc;

			rc = smb1360_masked_write(battery, CFG_CHG_MISC_REG,
						  CHG_CURR_TERM_DIS_BIT, 0);
			if (rc)
				return rc;
		}
	} else  if (battery->iterm_disabled) {
		rc = smb1360_masked_write(battery, CFG_CHG_MISC_REG,
					  CHG_CURR_TERM_DIS_BIT,
					  CHG_CURR_TERM_DIS_BIT);
		if (rc)
			return rc;
	}

	/* set the safety time voltage */
	if (battery->safety_time != -EINVAL) {
		if (battery->safety_time == 0) {
			/* safety timer disabled */
			rc = smb1360_masked_write(battery, CFG_SFY_TIMER_CTRL_REG,
						SAFETY_TIME_EN_BIT, 0);
			if (rc < 0) {
				dev_err(battery->dev,
				"couldn't disable safety timer: %d\n", rc);
				return rc;
			}
		} else {
			for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
				if (battery->safety_time <= chg_time[i]) {
					reg = i << SAFETY_TIME_MINUTES_SHIFT;
					break;
				}
			}
			rc = smb1360_masked_write(battery, CFG_SFY_TIMER_CTRL_REG,
				SAFETY_TIME_EN_BIT | SAFETY_TIME_MINUTES_MASK,
				SAFETY_TIME_EN_BIT | reg);
			if (rc < 0) {
				dev_err(battery->dev,
					"couldn't set safety timer: %d\n", rc);
				return rc;
			}
		}
	}

	/* configure resume threshold, auto recharge and charge inhibit */
	if (battery->resume_delta_mv != -EINVAL) {
		if (battery->recharge_disabled && battery->chg_inhibit_disabled) {
			dev_err(battery->dev, "Error: Both recharge_disabled and recharge_mv set\n");
			return -EINVAL;
		} else {
			rc = smb1360_recharge_threshold_set(battery,
						battery->resume_delta_mv);
			if (rc) {
				dev_err(battery->dev,
					"couldn't set rechg thresh: %d\n", rc);
				return rc;
			}
		}
	}

	rc = smb1360_masked_write(battery, CFG_CHG_MISC_REG,
					CFG_AUTO_RECHG_DIS_BIT,
					battery->recharge_disabled ?
					CFG_AUTO_RECHG_DIS_BIT : 0);
	if (rc) {
		dev_err(battery->dev, "couldn't set rechg-cfg: %d\n", rc);
		return rc;
	}
	rc = smb1360_masked_write(battery, CFG_CHG_MISC_REG,
					CFG_CHG_INHIBIT_EN_BIT,
					battery->chg_inhibit_disabled ?
					0 : CFG_CHG_INHIBIT_EN_BIT);
	if (rc) {
		dev_err(battery->dev, "couldn't set chg_inhibit: %d\n", rc);
		return rc;
	}

	/* battery missing detection */
	rc = smb1360_masked_write(battery, CFG_BATT_MISSING_REG,
				BATT_MISSING_SRC_THERM_BIT,
				BATT_MISSING_SRC_THERM_BIT);
	if (rc < 0) {
		dev_err(battery->dev, "couldn't set batt_missing: %d\n", rc);
		return rc;
	}

	/* interrupt enabling - active low */
	if (battery->client->irq) {
		mask = CHG_STAT_IRQ_ONLY_BIT | CHG_STAT_ACTIVE_HIGH_BIT
						| CHG_STAT_DISABLE_BIT;
		reg = CHG_STAT_IRQ_ONLY_BIT;
		rc = smb1360_masked_write(battery, CFG_STAT_CTRL_REG, mask, reg);
		if (rc < 0) {
			dev_err(battery->dev, "couldn't set irq: %d\n", rc);
			return rc;
		}

		/* enabling only interesting interrupts */
		rc = smb1360_write(battery, IRQ_CFG_REG, IRQ_CFG_FULL_CONF);
		if (rc) {
			dev_err(battery->dev, "couldn't set irq1: %d\n", rc);
			return rc;
		}

		rc = smb1360_write(battery, IRQ2_CFG_REG, IRQ2_CFG_FULL_CONF);
		if (rc) {
			dev_err(battery->dev, "couldn't set irq2: %d\n", rc);
			return rc;
		}

		rc = smb1360_write(battery, IRQ3_CFG_REG, IRQ3_CFG_FULL_CONF);
		if (rc < 0) {
			dev_err(battery->dev, "couldn't set irq3: %d\n", rc);
			return rc;
		}
	}

	if (battery->revision != SMB1360_REV_1)
		smb1360_fg_config(battery);

	return rc;
}

static int smb1360_parse_properties(struct smb1360_battery *battery)
{
	int ret = 0;
	struct device *dev = &battery->client->dev;

	ret = device_property_read_u32(dev, "qcom,float-voltage-mv",
				       &battery->vfloat_mv);
	if (ret < 0)
		battery->vfloat_mv = -EINVAL;

	ret = device_property_read_u32(dev, "qcom,charging-timeout",
				       &battery->safety_time);
	if (ret < 0)
		battery->safety_time = -EINVAL;

	if (!ret && (battery->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1]))
		return -EINVAL;

	ret = device_property_read_u32(dev, "qcom,recharge-thresh-mv",
				       &battery->resume_delta_mv);
	if (ret < 0)
		battery->resume_delta_mv = -EINVAL;

	ret = device_property_read_u32(dev, "qcom,iterm-ma",
				       &battery->iterm_ma);
	if (ret < 0)
		battery->iterm_ma = -EINVAL;

	battery->recharge_disabled =
		device_property_read_bool(dev, "qcom,recharge-disabled");
	battery->iterm_disabled =
		device_property_read_bool(dev, "qcom,iterm-disabled");
	battery->chg_inhibit_disabled =
		device_property_read_bool(dev, "qcom,chg-inhibit-disabled");

	/* fg params */
	ret = device_property_read_u32(dev, "qcom,fg-delta-soc",
				       &battery->delta_soc);
	if (ret < 0)
		battery->delta_soc = -EINVAL;

	ret = device_property_read_u32(dev, "qcom,fg-soc-max",
				       &battery->soc_max);
	if (ret < 0)
		battery->soc_max = -EINVAL;

	ret = device_property_read_u32(dev, "qcom,fg-soc-min",
				       &battery->soc_min);
	if (ret < 0)
		battery->soc_min = -EINVAL;

	ret = device_property_read_u32(dev, "qcom,fg-voltage-min-mv",
				       &battery->voltage_min_mv);
	if (ret < 0)
		battery->voltage_min_mv = -EINVAL;

	ret = device_property_read_u32(dev, "qcom,fg-voltage-empty-mv",
				       &battery->voltage_empty_mv);
	if (ret < 0)
		battery->voltage_empty_mv = -EINVAL;

	return 0;
}

static const struct regmap_config smb1360_battery_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
};

static const struct power_supply_desc smb1360_battery_desc = {
	.name			= "smb1360-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= smb1360_battery_get_property,
	.properties		= smb1360_battery_props,
	.num_properties		= ARRAY_SIZE(smb1360_battery_props),
	.external_power_changed	= smb1360_external_power_changed,
};

static int smb1360_probe(struct i2c_client *client)
{
	int ret;
	struct power_supply_config psy_cfg = {};
	struct smb1360_battery *battery;

	battery = devm_kzalloc(&client->dev, sizeof(*battery), GFP_KERNEL);
	if (!battery) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -EINVAL;
	}

	battery->client = client;

	ret = smb1360_parse_properties(battery);
	if (ret) {
		dev_err(&client->dev, "failed to parse dt properties\n");
		return ret;
	}

	battery->regmap = devm_regmap_init_i2c(client,
					       &smb1360_battery_regmap_config);
	if (IS_ERR(battery->regmap)) {
		dev_err(&client->dev, "failed to init regmap\n");
		return -EINVAL;
	}

	device_init_wakeup(battery->dev, 1);

	i2c_set_clientdata(client, battery);
	psy_cfg.drv_data = battery;

	battery->resume_completed = true;

	mutex_init(&battery->charging_disable_lock);

	ret = smb1360_hw_init(battery);
	if (ret < 0) {
		dev_err(&client->dev,
			"unable to intialize hardware: %d\n", ret);
		return ret;
	}

	ret = determine_initial_status(battery);
	if (ret < 0) {
		dev_err(&client->dev,
			"unable to determine init status: %d\n", ret);
		return ret;
	}

	battery->psy = power_supply_register(&client->dev,
					     &smb1360_battery_desc, &psy_cfg);
	if (IS_ERR(battery->psy)) {
		dev_err(&client->dev, "failed to register power supply\n");
		ret = PTR_ERR(battery->psy);
		return ret;
	}

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						smb1360_bat_irq, IRQF_ONESHOT,
						NULL, battery);
		if (ret) {
			dev_err(&client->dev,
				"request irq %d failed\n", client->irq);
			return ret;
		}

		enable_irq_wake(client->irq);
	}

	ret = smb1360_read_revision(battery, &battery->revision);
	if (ret)
		dev_err(battery->dev, "couldn't read revision: %d\n", ret);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id smb1360_match_table[] = {
	{ .compatible = "qcom,smb1360" },
	{ },
};
MODULE_DEVICE_TABLE(of, smb1360_match_table);
#endif

static struct i2c_driver smb1360_driver = {
	.driver	= {
		.name = "smb1360-chg-fg",
		.of_match_table = of_match_ptr(smb1360_match_table),
	},
	.probe_new = smb1360_probe,
};

module_i2c_driver(smb1360_driver);

MODULE_DESCRIPTION("SMB1360 Charger and Fuel Gauge");
MODULE_LICENSE("GPL v2");
