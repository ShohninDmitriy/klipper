Input Shaping and S-Curve acceleration
====================

Klipper has support of several features that can be used to reduce ghosting in
prints: Input Shaping and adaptive S-Curve acceleration.
[Input shaping](https://en.wikipedia.org/wiki/Input_shaping) is an open-loop
control technique which creates a commanding signal that cancels its own
vibrations. S-Curve acceleration scheme replaces the traditional
"[trapezoid generator](Kinematics.md#trapezoid-generator)" by modeling
higher-order derivatives, like jerk, snap and so forth, depending on
the S-Curve order, using Bezier polynomials.

**Warning**: Input Shaping and S-Curve acceleration support is experimental.
You should consider using them only if you already have some problems with the
prints, otherwise it is not advised to switch over from the default trapezoid
generator. Examples of problems S-Curve acceleration may help with:

* Ghosting and ringing in the prints
* Extruder skipping steps or rattling when using
  [Pressure Advance](Pressure_Advance.md):
    * bowden extruder with very high pressure advance value
    * direct extruder with high gear ratio (e.g. remote direct drive extruder)

The improvements do not come for free: adaptive S-Curve acceleration often
slows prints down. Depending on the model and print parameters, slow down can
be from negligible to 20-30% and more. On the other hand, the same level of
improvements usually cannot be obtained just by reducing the acceleration.

Note that ghosting usually has mechanical origins: insufficiently rigid printer
frame, non-tight or too springy belts, alignment issues of mechanical parts,
heavy moving mass, etc. Those should be checked and fixed first.


Switch to S-Curve acceleration branch
===========================

Instructions below assume that you have an existing Klipper installation
from the main repo. If you don't, follow the
[instructions](https://github.com/KevinOConnor/klipper/blob/master/docs/Installation.md)
to set up the Klipper from the main repo first.

To try experimental Input Shaping and S-Curve acceleration mode in your existing
Klipper installation, SSH to your Raspberry Pi and run the following commands:
```
$ cd klipper
$ sudo service klipper stop
```

Configure the new Git remote:
```
$ git remote add s-curve-exp https://github.com/dmbutyugin/klipper.git
$ git remote -v
```
The output should list the new remote among other things:
```
s-curve-exp	https://github.com/dmbutyugin/klipper.git (fetch)
s-curve-exp	https://github.com/dmbutyugin/klipper.git (push)
```

Now check the current branch, it will be needed to roll back after you are
finished with the experiments:
```
$ git branch
```
will most likely list
```
* master
```

Check out the new branch:
```
$ git fetch s-curve-exp
$ git checkout s-curve-exp/scurve-shaping
```

Start Klipper:
```
$ sudo service klipper start
```

If you want to switch back to the main Klipper branch, SSH to your Raspberry
Pi and run the following commands:
```
$ cd klipper
$ sudo service klipper stop
$ git checkout master
$ sudo service klipper start
```

## Updating

To update an existing installation to a newer version of the code, run the
following commands:
```
$ cd klipper
$ sudo service klipper stop
$ git fetch s-curve-exp
$ git checkout s-curve-exp/scurve-shaping
$ sudo service klipper start
```

Tuning
===========================

Basic tuning requires measuring the ringing frequencies of the printer and
adding a few parameters to `printer.cfg` file.


Slice the ringing test model, which can be found in
[docs/prints/ringing_tower.stl](prints/ringing_tower.stl), in the slicer:

 * Suggested layer height is 0.2 or 0.25 mm.
 * Infill and top layers can be set to 0.
 * Use 1-2 perimeters, or even better the smooth vase mode with 1-2 mm base.
 * Use sufficiently high speed, around 80-100 mm/sec, for *external* perimeters.
 * Make sure that the minimum layer time is *at most* 3 seconds.

## Ringing frequency

First, measure the **ringing frequency**. Note that these measurements can also
be done on the mainline Klipper branch before switching to the S-Curve branch.

1. Increase `max_accel` and `max_accel_to_decel` parameters in your
   `printer.cfg` to 7000.
2. Restart the firmware: `RESTART`.
3. Disable Pressure Advance: `SET_PRESSURE_ADVANCE ADVANCE=0`.
4. If you have already switched to the S-Curve branch and updated the config,
   execute `SET_SCURVE ACCEL_ORDER=2 MIN_ACCEL=7000` and
   `SET_INPUT_SHAPER SHAPER_FREQ_X=0 SHAPER_FREQ_Y=0` commands. If you get
   "Unknown command" errors for any of these commands, you can safely ignore
   them at this point and continue with the measurements.
5. Execute the command
   `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.
   Basically, we try to make ringing more pronounced by setting different large
   values for acceleration.
6. Print the test model sliced with the suggested parameters.
7. You can stop the print earlier if the ringing is clearly visible and you see
   that acceleration gets too high for your printer (e.g. printer shakes too
   much or starts skipping steps).
8. Measure the distance *D* (in mm) between *N* oscillations for X axis near
   the notches, preferably skipping the first oscillation or two. Pay attention
   to the notches X axis corresponds to - the test model has large 'X' and 'Y'
   marks on the back side for convenience. Note that 'X' mark is on Y axis and
   vice versa, it is not a mistake - ringing of X axis shows *along* Y axis.
   To measure the distance between oscillations more easily, mark the
   oscillations first, then measure the distance between the marks with a ruler
   or calipers:

    |![Mark ringing](img/ringing-mark.jpg)|![Measure ringing](img/ringing-measure.jpg)|
    |:--:|:--:|

9. Compute the ringing frequency = *V* &middot; *N* / *D* (Hz) where *V* is the
   velocity for outer perimeters (mm/sec). For the example above, we marked 6
   oscillations, and the test was printed at 100 mm/sec velocity, so the
   frequency is 100 * 6 / 12.14 ≈ 49.4 Hz.
10. Do (8) - (9) for Y axis as well.

Note that ringing on the test print should follow the pattern of the curved
notches, as in the picture above. If it doesn't, then this defect is not really
a ringing and has a different origin - either mechanical, or an extruder issue.
It should be fixed first before enabling and tuning input shapers.

If the measurements are not reliable because, say, the distance
between the oscillations is not stable, it might mean that the printer has
several resonance frequencies on the same axis. One may try to follow the
tuning process described in
[Unreliable measurements of ringing frequencies](#unreliable-measurements-of-ringing-frequencies)
section instead and still get something out of the input shaping technique.

For Cartesian printers, you will obtain 2 frequencies (f<sub>X</sub> and
f<sub>Y</sub>), which may be different, especially on bed-slinger printers.
Since CoreXY printers could have 4 frequencies, it is possible, but not
mandatory, to repeat the process (2)-(10) rotating the model 45 degrees around
Z axis such that the angle between the X and Y axes and the sides of the model
is 45 degrees; then measure 2 extra frequencies. Delta printers could have
resonances at different frequencies at different toolhead positions and in
different directions, however they often have a single strong resonance with a
fairly stable frequency; just measure 2 frequencies as for Cartesian printers
(they should be equal or close to each other).

Ringing frequency can depend on the position of the model within the buildplate
and Z height, *especially on delta printers*; you can check if you see the
differences in frequencies at different positions along the sides of the test
model and at different heights. You can calculate the average ringing
frequencies over X and Y axes if that is the case.

If the measured ringing frequency is very low (below approx 20-25 Hz), it might
be a good idea to invest into stiffening the printer or decreasing the moving
mass - depending on what is applicable in your case - before proceeding with
further input shaping tuning, and re-measuring the frequencies afterwards. For
many popular printer models there are often some solutions available already.

Note that the ringing frequencies can change if the changes are made to the
printer that affect the moving mass or change the stiffness of the system,
for example:

  * Some tools are installed, removed or replaced on the toolhead that change
    its mass, e.g. a new (heavier or lighter) stepper motor for direct extruder
    or a new hotend is installed, heavy fan with a duct is added, etc.
  * Belts are tightened.
  * Some addons to increase frame rigidity are installed.
  * Different bed is installed on a bed-slinger printer, or glass added, etc.

If such changes are made, it is a good idea to at least measure the ringing
frequencies to see if they have changed.

## Input shaper configuration

After the ringing frequencies for X and Y axes are measured, you can add the
following section to your `printer.cfg`:
```
[input_shaper]
shaper_freq_x: ...
shaper_freq_y: ...
```

For the example above, we get shaper_freq_x/y = 49.4.

## Choosing input shaper

Klipper supports several input shapers. They differ in their sensitivity to
errors determining the resonance frequency and how much smoothing they cause
in the printed parts. Also, some of the shapers like 2HUMP_EI and 3HUMP_EI
should usually not be used with shaper_freq = resonance frequency - they are
configured from different considerations to reduce several resonances at once.

For most of the printers, either MZV or EI shapers can be recommended. This
section describes a testing process to choose between them, and figure out
a few other related parameters.

Print the ringing test model as follows (assuming you already have
shaper_freq_x/y set and max_accel/max_accel_to_decel increased to 7000 in
printer.cfg file):

1. Restart the firmware: `RESTART`.
2. Disable Pressure Advance: `SET_PRESSURE_ADVANCE ADVANCE=0`.
3. Execute `SET_SCURVE ACCEL_ORDER=2 MIN_ACCEL=7000` (ignore any errors).
4. Execute `SET_INPUT_SHAPER SHAPER_TYPE=MZV`.
5. Execute the command
   `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.
6. Print the test model sliced with the suggested parameters.

If you see no ringing at this point, then MZV shaper can be recommended for use.

If you do see some ringing, re-measure the frequencies using steps (8)-(10)
described in [Ringing frequency](#ringing-frequency) section. If the frequencies
differ significantly from the values you obtained earlier, a more complex input
shaper configuration is needed. You can refer to Technical details of
[Input shapers](#input-shapers) section. Otherwise, proceed to the next step.

Now try EI input shaper. To try it, repeat steps (1)-(6) from above, but
executing at step 4 the following command instead:
`SET_INPUT_SHAPER SHAPER_TYPE=EI`.

Compare two prints with MZV and EI input shaper. If EI shows noticeably better
results than MZV, use EI shaper, otherwise prefer MZV. Note that EI shaper will
cause more smoothing in printed parts (see the next section for further
details). Add `shaper_type: mzv` (or ei) parameter to [input_shaper] section,
e.g.:
```
[input_shaper]
shaper_freq_x: ...
shaper_freq_y: ...
shaper_type: mzv
```

A few notes on shaper selection:

  * EI shaper may be more suited for bed slinger printers (if the resonance
    frequency and resulting smoothing allows): as more filament is deposited
    on the moving bed, the mass of the bed increases and the resonance frequency
    will decrease. Since EI shaper is more robust to resonance frequency
    changes, it may work better when printing large parts.
  * Due to the nature of delta kinematics, resonance frequencies can differ a
    lot in different parts of the build volume. Therefore, EI shaper can be a
    better fit for delta printers rather than MZV or ZV, and should be
    considered for the use. If the resonance frequency is sufficiently large
    (more than 50-60 Hz), then one can even attempt to test 2HUMP_EI shaper
    (by running the suggested test above with
    `SET_INPUT_SHAPER SHAPER_TYPE=2HUMP_EI`), but check the considerations in
    the [section below](#selecting-max_accel) before enabling it.

## Selecting max_accel

You should have a printed test for the shaper you chose from the previous step.
Note that at very high accelerations, depending on the resonance frequency and
the input shaper you chose (e.g. EI shaper creates more smoothing than MZV),
input shaping may cause too much smoothing and rounding of the parts. So,
max_accel should be chosen such as to prevent that. Another parameter that can
impact smoothing is square_corner_velocity, so it is not advisable to increase
it above the default 5 mm/sec to prevent increased smoothing.

Inspect the model for the chosen input shaper. Take a note at which
acceleration:

  * ringing is still small - that you are comfortable with it;
  * corners of the test model still show 'bulges' from disabled pressure
    advance - remember that **input shaping is not a replacement of PA**.

Choose the minimum out of the two acceleration values, and put it as max_accel
into printer.cfg (you can delete max_accel_or_decel or revert it to the old
value).

As a note, it may happen - especially at low ringing frequencies - that EI
shaper will cause too much smoothing even at lower accelerations. In this case,
MZV may be a better choice, because it may allow higher acceleration values.

At very low ringing frequencies (~25 Hz and below) even MZV shaper may create
too much smoothing. If that is the case, you can also try to repeat the
steps in [Choosing input shaper](#choosing-input-shaper) section with ZV shaper,
by using `SET_INPUT_SHAPER SHAPER_TYPE=ZV` command instead. ZV shaper should
show even less smoothing than MZV, but is more sensitive to errors in measuring
the ringing frequencies.

Another consideration is that if a resonance frequency is too low (below 20-25
Hz), it might be a good idea to increase the printer stiffness or reduce the
moving mass. Otherwise, acceleration and printing speed may be limited due too
much smoothing now instead of ringing.

## S-Curve configuration

If you see no ringing on the test model printed with your chosen input shaper,
you do not need to configure S-Curve acceleration. If you still see some ringing
up to and a bit above the max_accel value you chose, you may try S-Curve
acceleration. Add the following section to printer.cfg file (and restart
Klipper afterwards):
```
[scurve]
acceleration_order: 4
min_accel: ...
```

As `min_accel` choose the value of acceleration from the test model when there is
still no ringing visible. The tuning is complete.


## Fine-tuning resonance frequencies

Note that the precision of the resonance frequencies measurements using the
ringing test model is sufficient for most purposes, so further tuning is not
advised. If you still want to try to double-check your results (e.g. if you
still see some ringing after printing a test model with an input shaper of
your choice with the same frequencies as you have measured earlier), you can
follow the steps in this section. Note that if you see ringing at different
frequencies after enabling [input_shaper], this section will not help with that.

Assuming that you have sliced the ringing model with suggested parameters and
increased `max_accel` and `max_accel_to_decel` parameters in the `printer.cfg`
to 7000 already, complete the following steps for each of the axes X and Y:

1. Make sure Pressure Advance is disabled: `SET_PRESSURE_ADVANCE ADVANCE=0`.
2. Execute `SET_INPUT_SHAPER SHAPER_TYPE=ZV`.
2. From the existing ringing test model with your chosen input shaper select
   the acceleration that shows ringing sufficiently well, and set it with:
   `SET_VELOCITY_LIMIT ACCEL=...` and
   `SET_SCURVE ACCEL_ORDER=2 MIN_ACCEL=...` (ignore any errors).
4. Calculate the necessary parameters for the `TUNING_TOWER` command to tune
   `shaper_freq_x` parameter as follows: start = shaper_freq_x * 83 / 132 and
   factor = shaper_freq_x / 66, where `shaper_freq_x` here is the current value
   in `printer.cfg`.
5. Execute the command
   `TUNING_TOWER COMMAND=SET_INPUT_SHAPER PARAMETER=SHAPER_FREQ_X START=start FACTOR=factor BAND=5`
   using `start` and `factor` values calculated at step (4).
6. Print the test model.
7. Reset the original frequency value:
   `SET_INPUT_SHAPER SHAPER_FREQ_X=...`.
7. Find the band which shows ringing the least and count its number from the
   bottom starting at 1.
8. Calculate the new shaper_freq_x value via old
   shaper_freq_x * (39 + 5 * #band-number) / 66.

Repeat these steps for the Y axis in the same manner, replacing references to X
axis with the axis Y (e.g. replace `shaper_freq_x` with `shaper_freq_y` in
the formulae and in the `TUNING_TOWER` command).

As an example, let's assume you have had measured the ringing frequency for one
of the axis equal to 45 Hz. This gives start = 45 * 83 / 132 = 28.30
and factor = 45 / 66 = 0.6818 values for `TUNING_TOWER` command.
Now let's assume that after printing the test model, the fourth band from the
bottom gives the least ringing. This gives the updated shaper_freq_? value
equal to 45 * (39 + 5 * 4) / 66 ≈ 40.23.

After both new `shaper_freq_x` and `shaper_freq_y` parameters have been
calculated, you can update `[input_shaper]` section in `printer.cfg` with the
new `shaper_freq_x` and `shaper_freq_y` values.

## Pressure Advance

If you use Pressure Advance, it may need to be re-tuned. Follow the
[instructions](Pressure_Advance.md#tuning-pressure-advance) to find the
new value, if it differs from the previous one. Make sure to restore the
original values of `max_accel` and `max_accel_to_decel` parameters in the
`printer.cfg` and restart Klipper before tuning Pressure Advance.

If later during printing you notice that extruder rattles or skip steps, it
means that the pressure advance value is too high for the corresponding
acceleration. You can either reduce `max_accel` setting, or reduce `min_accel`
value in `[scurve]` section.


## Unreliable measurements of ringing frequencies

If you are unable to measure the ringing frequencies, e.g. if the distance
between the oscillations is not stable, you may still be able to take advantage
of input shaping techniques, but the results may not be as good as with proper
measurements of the frequencies, and will require a bit more tuning and printing
the test model. Note that another possibility is to purchase and install an
accelerometer and measure the resonances with it (there is a separate
[branch](https://github.com/dmbutyugin/klipper/tree/adxl345-spi) with ADXL345
support) - but this option requires some crimping and soldering.


For tuning, add empty `[input_shaper]` section to your `printer.cfg` (do not add
`[scurve]` section yet, or remove if you had any). Then, assuming that you have
sliced the ringing model with suggested parameters and increased `max_accel` and
`max_accel_to_decel` parameters in the `printer.cfg` to 7000 already, print the
test model 3 times as follows. First time, prior to printing, run

1. `RESTART`
2. `SET_PRESSURE_ADVANCE ADVANCE=0`.
3. `SET_INPUT_SHAPER SHAPER_TYPE=2HUMP_EI SHAPER_FREQ_X=60 SHAPER_FREQ_Y=60`.
4. `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.

and print the model. Then print the model again, but before printing run instead

1. `SET_INPUT_SHAPER SHAPER_TYPE=2HUMP_EI SHAPER_FREQ_X=50 SHAPER_FREQ_Y=50`.
2. `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.

Then print the model for the 3rd time, but now run

1. `SET_INPUT_SHAPER SHAPER_TYPE=2HUMP_EI SHAPER_FREQ_X=40 SHAPER_FREQ_Y=40`.
2. `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.

Essentially, we are printing the ringing test model with TUNING_TOWER using
2HUMP_EI shaper with shaper_freq = 60 Hz, 50 Hz, and 40 Hz.

If none of the models demonstrate improvements in ringing, then, unfortunately,
it does not look like the input shaping techniques can help with your case.

Otherwise, it may be that all models show no ringing, or some show the ringing
and some - not so much. Choose the test model with the highest frequency that
still shows good improvements in ringing. For example, if 40 Hz and 50 Hz models
show almost no ringing, and 60 Hz model already shows some more ringing, stick
with 50 Hz.

Now check if EI shaper would be good enough in your case. Choose EI shaper
frequency based on the frequency of 2HUMP_EI shaper you chose:

  * For 2HUMP_EI 60 Hz shaper, use EI shaper with shaper_freq = 50 Hz.
  * For 2HUMP_EI 50 Hz shaper, use EI shaper with shaper_freq = 40 Hz.
  * For 2HUMP_EI 40 Hz shaper, use EI shaper with shaper_freq = 33 Hz.

Now print the test model one more time, running

1. `SET_INPUT_SHAPER SHAPER_TYPE=EI SHAPER_FREQ_X=... SHAPER_FREQ_Y=...`.
2. `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.

providing the shaper_freq_x=... and shaper_freq_y=... as determined previously.

If EI shaper shows very comparable good results as 2HUMP_EI shaper, stick with
EI shaper and the frequency determined earlier, otherwise use 2HUMP_EI shaper
with the corresponding frequency. Add the results to `printer.cfg` as, e.g.
```
[input_shaper]
shaper_freq_x: 50
shaper_freq_y: 50
shaper_type: 2hump_ei
```

Continue the tuning with [Selecting max_accel](#selecting-max_accel) section.


Troubleshooting and FAQ
=======================

### I cannot get reliable measurements of resonance frequencies

First, make sure it is not some other problem with the printer instead of
ghosting. If the measurements are not reliable because, say, the distance
between the oscillations is not stable, it might mean that the printer has
several resonance frequencies on the same axis. One may try to follow the
tuning process described in
[Unreliable measurements of ringing frequencies](#unreliable-measurements-of-ringing-frequencies)
section and still get something out of the input shaping technique.

### After enabling [input_shaper], I get too smoothed printed parts and fine details are lost

Check the considerations in [Selecting max_accel](#selecting-max_accel) section.
If the resonance frequency is low, one should not set too high max_accel or
increase square_corner_velocity parameters. It might also be better to choose
MZV or even ZV input shapers over EI (or 2HUMP_EI and 3HUMP_EI shapers).


### After successfully printing for some time without ghosting, it appears to come back

It is possible that after some time the resonance frequencies have changed.
E.g. maybe the belts tension has changed (belts got more loose), etc. It is a
good idea to check and re-measure the ringing frequencies as described in
[Ringing frequency](#ringing-frequency) section and update your config file
if necessary.

### Is dual carriage setup supported with input shapers?

There is no dedicated support for dual carriages with input shapers, but it does
not mean this setup will not work. One should run the tuning twice for each
of the carriages, and calculate the ringing frequencies for X and Y axes for
each of the carriages independently. Then put the values for carriage 0 into
[input_shaper] section, and change the values on the flight when changing
carriages, e.g. as a part of some macro:
```
SET_DUAL_CARRIAGE CARRIAGE=1
SET_INPUT_SHAPER SHAPER_FREQ_X=... SHAPER_FREQ_Y=...
```

And similarly when switching back to carriage 0.


Technical details
=================

## Input shapers

Input shapers used in Klipper are rather standard, and one can find more
in-depth overview in the articles describing the corresponding shapers.
This section contains a brief overview of some technical aspects of the
supported input shapers. The table below shows some (usually approximate)
parameters of each shaper.

| Input <br> shaper | Shaper <br> duration | Vibration reduction 20x <br> (5% vibration tolerance) | Vibration reduction 10x <br> (10% vibration tolerance) |
|:--:|:--:|:--:|:--:|
| ZV | 0.5 / shaper_freq | N/A | ± 5% shaper_freq |
| MZV | 0.75 / shaper_freq | ± 4% shaper_freq | -10%...+15% shaper_freq |
| ZVD | 1 / shaper_freq | ± 15% shaper_freq | ± 22% shaper_freq |
| EI | 1 / shaper_freq | ± 20% shaper_freq | ± 25% shaper_freq |
| 2HUMP_EI | 1.5 / shaper_freq | ± 35% shaper_freq | ± 40 shaper_freq |
| 3HUMP_EI | 2 / shaper_freq | -45...+50% shaper_freq | -50%...+55% shaper_freq |

A note on vibration reduction: the values in the table above are approximate.
If the damping ratio of the printer is known for each axis, the shaper can be
configured more precisely and it will then reduce the resonances in a bit wider
range of frequencies. However, the damping ratio is usually unknown and is hard
to estimate without a special equipment, so Klipper uses 0.1 value by default,
which is a good all-round value. The frequency ranges in the table cover a
number of different possible damping ratios around that value (approx. from 0.05
to 0.2).

Also note that EI, 2HUMP_EI, and 3HUMP_EI are tuned to reduce vibrations to 5%,
so the values for 10% vibration tolerance are provided only for the reference.

**How to use this table:**

  * Shaper duration affects the smoothing in parts - the larger it is, the more
    smooth the parts are. This dependency is not linear, but can give a sense of
    which shapers 'smooth' more for the same frequency. The ordering by
    smoothing is like this: ZV < MZV < ZVD ≈ EI < 2HUMP_EI < 3HUMP_EI. Also,
    it is rarely practical to set shaper_freq = resonance freq for shapers
    2HUMP_EI and 3HUMP_EI (they should be used to reduce vibrations for several
    frequencies).
  * One can estimate a range of frequencies in which the shaper reduces
    vibrations. For example, MZV with shaper_freq = 35 Hz reduces vibrations
    to 5% for frequencies [33.6, 36.4] Hz. 3HUMP_EI with shaper_freq = 50 Hz
    reduces vibrations to 5% in range [27.5, 75] Hz.
  * One can use this table to check which shaper they should be using if they
    need to reduce vibrations at several frequencies. For example, if one has
    resonances at 35 Hz and 60 Hz on the same axis: a) EI shaper needs to have
    shaper_freq = 35 / (1 - 0.2) = 43.75 Hz, and it will reduce resonances
    until 43.75 * (1 + 0.2) = 52.5 Hz, so it is not sufficient; b) 2HUMP_EI
    shaper needs to have shaper_freq = 35 / (1 - 0.35) = 53.85 Hz and will
    reduce vibrations until 53.85 * (1 + 0.35) = 72.7 Hz - so this is an
    acceptable configuration. Always try to use as high shaper_freq as possible
    for a given shaper (perhaps with some safety margin, so in this example
    shaper_freq ≈ 50-52 Hz would work best), and try to use a shaper with as
    small shaper duration as possible.
  * If one needs to reduce vibrations at several very different frequencies
    (say, 30 Hz and 100 Hz), they may see that the table above does not provide
    enough information. In this case one may have more luck with
    [scripts/graph_shaper.py](../scripts/graph_shaper.py) script, which is more
    flexible.

S-Curve acceleration overview
=============================

By default, Klipper uses "trapezoid generator" for each move - each move has
a start speed, it accelerates to a cruising speed at constant acceleration,
it cruises at a constant speed, and then decelerates to the end speed using
constant acceleration.

![trapezoid](img/trapezoid.svg.png)

The basic idea behind S-Curve acceleration is to replace the constant
acceleration and deceleration with polynomials of higher order. For some
choices of the polynomials the toolhead will travel the same path and reach
the same speed as if it was accelerating constantly, but higher order
derivatives (e.g. acceleration, jerk) can be 0 in the beginning and at the end
of acceleration depending on the polynomial order, ensuring better smoothness
of the motion.

Klipper currently supports acceleration_order = 2, 4 and 6.
acceleration_order = 2 is the default constant acceleration mode.
Charts below show the distance covered, velocity and acceleration for different
acceleration orders.

|![Distance](img/s-curve-x.svg.png)|
|:--:|
| *Distance* |
|![Velocity](img/s-curve-v.svg.png)|
| *Velocity* |
|![Acceleration](img/s-curve-a.svg.png)|
| *Acceleration* |

Notice that the velocity and acceleration are smoother with higher acceleration
orders. When acceleration is not constant, the 'lack' of acceleration in
the beginning must be compensated with higher max acceleration. Klipper still
plans all moves considering the 'effective' acceleration, which can be seen as
an average acceleration, but then each move is executed using the polynomial
of the chosen degree. Though instantaneous acceleration exceeds the configured
maximum toolhead acceleration, because the movement is smoother overall,
the reduction of the maximum permitted acceleration is usually not necessary.

Besides making the movements smoother, S-Curve acceleration *may* improve the
quality of the prints. One of the theories behind it is that each printer has
limited non-infinite rigidity of the frame, belts, etc. When the force is
applied or relieved instantly in the beginning and at the end of acceleration
or deceleration, the system can act as a spring and start oscillating, which
can be observed in the form of ringing in the prints. S-Curve acceleration
'spreads' increase and decrease of the force during a longer time, potentially
reducing the oscillations.

This has an important consequence: if the short moves are executed with the
same acceleration, the full force must be applied over the shorter period of
time, effectively nullifying the positive effect of S-Curve acceleration on
short moves. That's why Klipper also limits the maximum kinematic jerk
*J* = *da* / *dt* of each acceleration and deceleration in S-Curve acceleration
mode.

For the chosen polynomials, the maximum kinematic jerk *J* is

*J* = 6 *a* / *T*

for acceleration_order = 4, where *a* is effective acceleration and *T* is
acceleration time. For acceleration_order = 6

*J* = 10 *a* / (*T* &#8730;3) &asymp; 5.774 *a* / *T*.

In the end, we use 6 *a* / *T* < max_jerk condition to limit the jerk. This
leads to a cubic equation

(*v*<sup>2</sup> - *v*<sub>0</sub><sup>2</sup>) &times;
  (*v* + *v*<sub>0</sub>) / 2 = *L*<sup>2</sup> *J* / 3,

which can be solved using Cardano's formula to determine the velocity *v*
after acceleration with the maximum jerk *J* over the segment of length *L*.
The final velocity is chosen as a minimum out of that value and
*v*<sub>0</sub> + (2 *a* *L*)<sup>1/2</sup>.

Another reason to limit the jerk is that it directly translates into extruder
acceleration *a*<sub>e</sub> if Pressure Advance is enabled:
*a<sub>e</sub>* = *r P J*,
where *P* is the pressure advance parameter,

*r* = (4 *w* *h*) / (&pi; *D*<sup>2</sup>).

is the extrusion ratio, *D* is the filament diameter, *w* is the extrusion
width, *h* is the layer height. As an example, with *P* = 0.5, *w* = 0.4 mm,
*h* = 0.2 mm, *D* = 1.75 mm, and *J* = 100'000 the extruder acceleration is
1663 mm/sec^2 due to jerk.

Extruder kinematics looks as follows with different acceleration orders:

|![Velocity](img/s-curve-ev.svg.png)|
|:--:|
| *Extruder Velocity* |
|![Acceleration](img/s-curve-ea.svg.png)|
| *Extruder Acceleration* |

Notice the velocity jump with acceleration_order = 2 (the 'infinite'
acceleration spikes at the beginning and the end of acceleration with
acceleration_order = 2 are not shown). With acceleration_order > 2
the velocity is continuous, and for acceleration_order = 6 it is even
smooth.  Thus, acceleration_order > 2 can improve the performance of
the extruder if pressure advance is enabled.