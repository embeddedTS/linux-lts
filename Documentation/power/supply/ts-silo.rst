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
