.. SPDX-License-Identifier: GPL-2.0-only

embeddedTS supervisory-controller supercapacitors
================================================

The TS-7100 and TS-7180 controller interface uses 16-bit big-endian register
addresses, byte status registers, and big-endian voltage/current words. This
is distinct from the Wizard MFD register map used by tssilo_supercaps.c.
The MFD parent provides a shared regmap and platform children. The SILO
driver selects the controller-specific operations while leaving the existing
Wizard ABI unchanged.

The supply is named ``silo``. Capacity is the voltage-based estimate used by
tsmicroctl: 0% at 3680 mV and 100% at 4800 mV, linearly interpolated and
clamped. It is not a calibrated energy or remaining-runtime measurement.
Standard voltage and current properties use microvolts and microamps.
Charging current accepts whole milliamps from 50 through 900 mA.
The current property controls the live setting. The controller also has a
persistent startup-current register, which this driver does not yet expose.
Disabling charging restores the live setting from that persistent value.

The controller has no programmable capacity alert threshold. Consequently,
capacity_alert_min is not exposed. Polling produces a power-supply change
event every second, allowing userspace to evaluate its own critical-capacity
policy. Userspace must not assume an unsupported attribute exists.

The controller's charge-enable operation also clears its minimum-charge
latch, matching tsmicroctl. Do not run a userspace controller writer concurrently
with this driver. The watchdog uses the same I2C address; a second enabled
I2C node at that address is not supported. Integration of both functions should
attach to this MFD parent rather than create a second I2C client. The
watchdog is not yet supported by this parent.

Older controller support
------------------------

TS-4100 and TS-7553-V2 use unaddressed I2C snapshots and one-byte charger
commands. Their capacity estimate follows the legacy tsmicroctl calculation; external
power status comes from the power-fail GPIO. The firmware provides no charger
state readback, so reading charge_behaviour returns ENODATA. Writes send the
requested enable or inhibit command, without claiming that it took effect.
Silod may log a readback warning after a successful charge-enable write.
Status is Unknown while external power is present and Discharging when it is
absent. On TS-7553-V2, firmware updates the supercapacitor ADC snapshot only
while SILO is enabled, so disabling it can leave stale readings. This implementation
has not yet been tested on TS-4100 or TS-7553-V2 hardware.

On TS-4100, a charger enable or inhibit command writes a single byte with
bit 1 clear. The controller interprets bit 1 as push-switch reset enable,
so either operation disables that function. This is intentional. The
microcontroller watchdog and power-off functions are not supported by the
MFD parent; the previously disabled watchdog node is replaced by the parent.
