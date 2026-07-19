# Project: Single-Joint Robotic Inspection Light Prototype

## 1. Purpose

Build a constrained mechanical/electrical prototype for a miniature robotic inspection light.

This prototype is **not** the full product. It is a test article for one remotely actuated arm joint:

- Base contains controller, motor, driver, buttons, and power.
- A rigid fixed arm extends approximately 10 inches from the base.
- A second 10-inch moving arm rotates about a single elbow joint.
- The elbow is driven remotely from the base using a toothed timing belt and timing pulleys.
- The joint must be back-drivable/protected so the user can manually hijack the arm without breaking the mechanism.
- Position is trusted only after homing against a contact switch.
- No encoder is used.

The long-term product concept is a “miniature robotic inspection light”: a placeable base with an articulated Luxo-like arm, camera, and flashlight head that can eventually point/peek toward a user-indicated target.

This project only proves the **single remote-driven elbow joint**.

---

## 2. Prototype Summary

Mechanical layout:

```text
[Base: motor + TMC5130 + controller]
        |
        |  fixed rigid arm, ~10"
        |
        O  elbow joint with large timing pulley + friction hub
         \
          \
           \ moving arm, ~10"
            \
             [dummy load representing future light/camera head]
```

Controlled degree of freedom:

```text
elbow angle θ
```

Target travel:

```text
0° to approximately 135°
```

Homing:

```text
single home contact/microswitch at one angular limit
```

Position model:

```text
after homing: step-counted position is valid
after manual movement/slip: position is invalid until re-homed
```

---

## 3. Design Goals

### Required

- Use off-the-shelf parts wherever possible.
- Use a TMC5130-based stepper driver/module/eval board.
- Use a NEMA stepper motor in the base.
- Use a timing belt and timing pulleys to drive the elbow.
- Use a home contact switch only; no encoder.
- Include mechanical hard stops beyond software travel limits.
- Include a friction hub/clutch at the elbow so forced manual movement slips safely.
- Provide a `HOME` button to reestablish position.
- Provide a `MANUAL` mode or release button that disables/reduces motor current and invalidates position.
- Firmware shall not rely on StallGuard for state validity or safety.

### Preferred

- 24 V motor supply.
- NEMA 17 motor for first prototype margin.
- GT2 9 mm belt for easy sourcing, or HTD 3M if stronger drive is needed.
- 20T motor pulley and 60T or 80T elbow pulley.
- 3:1 or 4:1 reduction.
- 10-inch fixed arm and 10-inch moving arm.
- Adjustable belt tensioner.
- Adjustable friction clutch preload.
- Dummy end load of 250–500 g.

### Explicitly Out of Scope

- Camera tracking.
- Flashlight/LED control.
- Pointer/finger detection.
- Multi-joint coordination.
- Wireless control.
- Battery optimization.
- Custom PCB.
- Encoder feedback.
- StallGuard-based slip detection.

---

## 4. System Architecture

### 4.1 Electrical Blocks

```text
MCU dev board
    |
    | SPI
    v
TMC5130 stepper driver/module
    |
    v
NEMA stepper motor

Inputs:
    HOME switch/contact
    HOME/RESUME button
    MANUAL button or toggle
    STOP button optional

Outputs:
    status LED(s)
    optional serial debug console
```

Recommended status LED meanings:

```text
blue    = homing
green   = homed / robotic mode enabled
yellow  = manual mode / position invalid
red     = stopped or fault
```

### 4.2 Mechanical Blocks

```text
Base:
    heavy enough to resist tipping
    holds motor, controller, driver, power, buttons
    contains belt tensioner if practical

Fixed arm:
    rigid 10-inch link from base to elbow
    carries timing belt along its length
    can be aluminum extrusion, aluminum flat bar, or dual side plates

Elbow:
    shaft supported by bearings in fixed arm
    large timing pulley coaxial with elbow
    friction clutch between pulley and moving arm hub
    home switch/contact
    mechanical stops

Moving arm:
    rigid 10-inch arm fixed to friction hub
    carries dummy mass at end
```

---

## 5. Mechanical Design Details

### 5.1 Timing Belt Drive

Use a toothed timing belt, also called:

```text
timing belt
synchronous belt
toothed belt
GT2 belt
HTD belt
```

The motor pulley and elbow pulley must match the belt tooth profile.

Recommended first version:

```text
belt:           GT2, 9 mm wide
motor pulley:   20T GT2, 5 mm bore for NEMA 17 motor
elbow pulley:   80T GT2, 8 mm bore preferred
ratio:          4:1
```

Alternative:

```text
motor pulley:   20T
elbow pulley:   60T
ratio:          3:1
```

Do not use worm gears or high-ratio gearboxes. The joint must remain mechanically forgiving and reasonably back-drivable.

### 5.2 Elbow Pulley / Moving Arm Relationship

The elbow pulley must drive the moving arm through a friction interface.

Normal operation:

```text
belt drives elbow pulley
elbow pulley drives moving arm through friction hub
```

Manual hijack:

```text
user forces moving arm
friction hub slips
belt/motor/structure are protected
position becomes invalid
user must press HOME before robotics resume
```

### 5.3 Friction Hub / Clutch

Use a simple adjustable friction clutch, not a precision commercial clutch.

Conceptual stack:

```text
fixed arm bearing plate
bearing
elbow shaft / shoulder bolt
large timing pulley
friction washer
moving arm hub plate
Belleville washer or spring washer
nylock nut / locknut / adjustable clamp screw
```

The moving arm should not be permanently rigid to the timing pulley. It should be coupled by adjustable friction.

Candidate friction materials:

```text
fiber washer
phenolic washer
rubberized cork washer
thin leather washer
urethane friction sheet
brake/clutch lining sheet
```

Use Belleville washers or spring washers to maintain preload.

Tune slip torque empirically.

Approximate tuning target:

```text
slip force at arm tip: 2–4 lbf initial target
arm length: 10 in
```

This corresponds roughly to:

```text
torque = force × radius
2 lbf × 10 in = 20 lbf·in
4 lbf × 10 in = 40 lbf·in
```

Tune by hanging weights from the arm tip and adjusting clutch preload.

### 5.4 Hard Stops

Include hard mechanical stops slightly beyond the software limits.

Example:

```text
home angle:             0°
software travel:        0° to 135°
mechanical stop range:  -5° to 145°
```

Hard stops must prevent:

- belt overtravel
- wire strain
- arm inversion
- collision with base
- damage to home switch

### 5.5 Prototype A Mechanical Architecture

Use a modular bench-test architecture instead of trying to make the first prototype compact.

Primary layout:

```text
top view:

        outboard belt plane
        |
        v

 [motor plate] 20T pulley ===== GT2 belt ===== 80T elbow pulley + clutch
       |                                           |
       |                                           O elbow shaft
 [base plate] ---- 2020 fixed-arm spine ---------- |
                                                   \
                                                    \
                                                     2020 moving arm
                                                      \
                                                       dummy load
```

The first mechanical build should be split into five modules:

```text
1. base module
2. fixed-arm spine
3. motor and belt-tension module
4. elbow bearing / pulley / clutch module
5. moving-arm and dummy-load module
```

#### 5.5.1 Base Module

The base should be a flat, heavy, drillable plate. A steel plate is preferred for the bench prototype because mass is useful and the base is not a product enclosure yet.

Base module responsibilities:

```text
hold the fixed arm rigidly
hold the motor bracket in the belt plane
provide belt tension adjustment
provide room for electronics and buttons
resist tipping with 250-500 g at the moving-arm tip
```

Recommended first build:

```text
base plate:        12 in x 12 in x 1/4 in steel
fixed arm mount:   2020 corner brackets plus backup angle/plate if needed
motor mount:       slotted NEMA 17 bracket or slotted flat plate
ballast:           5 lb weight plate or equivalent
```

Place the motor so the 20T pulley is in the same outboard plane as the 80T elbow pulley. Do not hide the belt inside the arm for Prototype A. The belt should be visible and accessible.

#### 5.5.2 Fixed-Arm Spine

Use one 10 in length of 2020 extrusion as the fixed arm spine. The extrusion is not the bearing support by itself; it is the straight, stiff beam that carries bolt-on plates.

Fixed-arm construction:

```text
2020 extrusion spine
flat aluminum base adapter plate or angle brackets
two aluminum elbow cheek plates at the distal end
optional printed belt guard after belt alignment is proven
```

The elbow cheek plates should bolt to the 2020 extrusion and carry two 8 mm flanged bearings. Keep the bearing spacing as wide as practical within the plate layout, roughly 25-40 mm, to reduce shaft wobble.

#### 5.5.3 Belt Plane And Tensioning

Use an outboard belt plane on one side of the fixed arm:

```text
20T motor pulley
closed-loop 9 mm GT2 belt
80T elbow pulley
optional toothed idler on a slotted tension plate
```

The simplest tension adjustment is a slotted motor mount:

```text
motor bracket slots:  +/- 10 mm travel minimum
idler slot:           optional, useful if belt length is not exact
belt guard:           add only after final alignment
```

Design the belt plane so the belt clears:

```text
fixed-arm brackets
home switch
hard stop tabs
moving-arm hub plate
user fingers during normal tests
```

#### 5.5.4 Elbow Module

Use an outboard clutch stack so the clutch is accessible for tuning.

Recommended stack from fixed arm outward:

```text
fixed-arm left cheek plate
8 mm flanged bearing
spacer tube between cheek plates
fixed-arm right cheek plate
8 mm flanged bearing
spacer washer
80T pulley fixed to 8 mm shaft with set screw
fiber friction washer
moving-arm hub plate with 8 mm clearance hole
Belleville washer stack
8 mm shaft collar used as preload adjuster
```

Important details:

```text
the 80T pulley is fixed to the shaft
the moving-arm hub is not fixed to the shaft
the moving arm is driven only by friction between pulley and hub
the shaft collar sets clutch preload
the bearings carry radial load; the clutch preload should not crush them
```

If clutch preload causes bearing drag, add a spacer sleeve that carries axial compression through the shaft stack instead of through the bearing races.

#### 5.5.5 Moving Arm Module

Use a second 10 in length of 2020 extrusion or aluminum flat bar as the moving arm. Bolt it to the moving-arm hub plate with at least two fasteners spaced apart.

Moving-arm requirements:

```text
length from elbow center to dummy load: about 10 in
dummy load:                            250-500 g
hub plate:                             flat aluminum, slotted if possible
wire routing:                          ignored for Prototype A unless adding a dummy cable loop
```

The dummy load should be adjustable along the arm for testing. Start with 250 g, then test 500 g after belt tracking and clutch slip are working.

#### 5.5.6 Home Switch And Hard Stops

Use a physical home switch and physical stops that are independent of firmware.

Recommended layout:

```text
moving hub plate has a small cam/tab
home switch mounts to fixed cheek plate on a slotted bracket
home switch actuates before the home hard stop
hard stop tab on moving hub contacts rubber bumper on fixed cheek
second hard stop limits maximum extension angle
```

Initial angle targets:

```text
home switch closes:     about +2 deg before the home hard stop
home hard stop:         about -5 deg
software zero:          0 deg after homing backoff and re-approach
max software angle:     135 deg
far hard stop:          about 145 deg
```

Make both stop positions adjustable in the first build. Fixed hard stops should be aluminum/steel tabs with rubber bumpers, not the microswitch body.

### 5.6 First-Order Mechanical Sizing

The simulation and build should start with these first-order numbers:

```text
moving arm length:      0.254 m / 10 in
dummy load:             0.25-0.50 kg
gravity torque at tip:  m * g * r
```

Worst-case dummy-load torque:

```text
0.50 kg * 9.81 m/s^2 * 0.254 m = 1.25 N*m
1.25 N*m ~= 11.1 lbf*in
```

Allowing for arm mass, bearing friction, and margin:

```text
expected working elbow torque:   1.5-2.0 N*m
clutch slip target:              2.3-4.5 N*m / 20-40 lbf*in
pulley ratio:                    4:1
motor torque before efficiency:  0.4-0.5 N*m for 1.5-2.0 N*m at elbow
```

This is consistent with using a NEMA 17 motor for the first prototype. If the arm cannot lift 500 g smoothly, reduce acceleration first, then increase motor current, then consider HTD 3M or a wider/stronger belt.

### 5.7 Simulation Recommendation

Use two simulation layers:

```text
1. Python calculator for sizing and firmware logic.
2. MuJoCo model for visual dynamic behavior.
```

Keep CAD separate:

```text
Onshape or Fusion 360:  geometry, interferences, bearing stack, belt length, drawings
MuJoCo:                 arm dynamics, gravity, actuator behavior, hard stops, manual disturbance
Python calculator:      torque, belt ratio, clutch threshold, step counts, homing state machine
```

Recommended first simulator: MuJoCo.

Reasons:

```text
free and open source
simple XML model format
good hinge-joint dynamics
joint limits and contacts are built in
actuators can apply torque through joints
Python control loop can model belt ratio, torque limits, clutch slip, and homing state
```

For this prototype, the MuJoCo model should not try to model every pulley tooth. Model the belt as an ideal 4:1 transmission:

```text
motor command -> 4:1 elbow joint actuator
motor-side torque limit -> elbow torque limit multiplied by ratio and efficiency
friction clutch -> torque limiter in the Python controller
manual hijack -> applied external torque that exceeds clutch threshold
hard stops -> elbow joint limits plus visible stop geometry
home switch -> angular threshold near the home limit
```

Minimal MuJoCo model contents:

```text
world/body:         fixed base
body:               fixed arm visual geometry
body:               moving arm
joint:              elbow hinge, range about -5 deg to 145 deg
geom:               dummy mass at moving-arm tip
actuator:           motor/position/torque actuator on elbow hinge
sensor or script:   home switch threshold
script state:       position_valid, homing, manual, clutch_slipped
```

Simulator options:

| Tool | Use it for | Fit for this prototype |
|---|---|---|
| MuJoCo | Dynamic behavior, gravity, torque limits, joint limits, scripted clutch slip | Best first dynamics simulator |
| Onshape | CAD, assembly mates, motion range, belt-length-driven layout, static checks | Best mechanical design workspace |
| PyBullet | Quick Python robotics simulation and collision experiments | Acceptable alternative, less clean for this clutch/transmission model |
| Drake | Formal multibody modeling and controls research | Too heavy for Prototype A unless the project grows into controls research |

The first useful simulator artifact should be a small Python/MuJoCo prototype with:

```text
slider: target elbow angle
button/key: HOME
button/key: MANUAL
disturbance input: simulated user forcing the arm
plot/log: elbow angle, motor target, clutch state, position_valid
```

---

## 6. Control Model

### 6.1 State Machine

Implement this deterministic state machine:

```text
POWER_ON
    motor disabled
    position invalid
    wait for HOME button

HOME_REQUESTED
    motor enabled at low current
    move slowly toward home switch

HOMING_CONTACT
    home switch/contact detected
    stop motor
    back off switch
    re-approach slowly
    set position = 0
    position valid

ROBOTIC_MODE
    step-counted motion allowed
    normal target commands accepted

MANUAL_MODE
    motor disabled or hold current reduced
    position invalid
    robotic motion disabled
    wait for HOME button

FAULT
    motor disabled
    position invalid
    wait for HOME button or reset
```

### 6.2 Position Validity Rule

The controller must enforce:

```text
Robotic motion is allowed only when position_valid == true.
```

Events that make position invalid:

```text
power-up
manual mode entered
STOP pressed
fault detected
optional: any driver error
optional: user command "invalidate"
```

Because there is no encoder, the controller must not pretend to know position after manual movement.

### 6.3 Manual Hijack Rule

The user may physically move the arm by forcing the friction hub to slip.

System behavior:

```text
if user intentionally wants manual movement:
    press MANUAL
    controller disables/reduces motor current
    position_valid = false
    user moves arm
    user presses HOME
    controller re-homes
    robotic mode resumes
```

If the user forces the arm without pressing MANUAL, the friction hub should still protect the mechanism. However, position may become wrong. User must press HOME before trusting robotic motion.

### 6.4 StallGuard

Do not use StallGuard as a safety mechanism or position-validity mechanism.

The TMC5130 is used for:

```text
stepper current control
quiet microstepping
ramp generation
position moves
```

Not for truth sensing.

---

## 7. Homing Sequence

Required homing sequence:

```text
1. Enable motor at low current.
2. Move slowly toward home direction.
3. When home contact closes, stop.
4. Back off until contact opens.
5. Pause briefly.
6. Approach slowly again.
7. On contact, set logical position = 0.
8. Back off to a safe ready offset if desired.
9. Set position_valid = true.
10. Enter ROBOTIC_MODE.
```

Use conservative speed and current during homing.

Homing should timeout if the switch is not reached within a plausible number of motor steps.

Timeout behavior:

```text
stop motor
position_valid = false
enter FAULT
```

---

## 8. Motion Parameters

Initial conservative settings:

```text
supply voltage:         24 V preferred
motor:                  NEMA 17 stepper
microstepping:          16 or 32
belt ratio:             4:1
max elbow speed:        slow, lamp-like
acceleration:           low
hold current:           low-to-moderate
run current:            enough to move without skipping under expected load
```

Avoid fast industrial robot motion. The prototype should move like a lamp:

```text
smooth
slow
non-threatening
quiet
```

---

## 9. Suggested Off-the-Shelf BOM

### Electronics

```text
MCU dev board:
    Arduino-class, ESP32, STM32, or similar

Stepper driver:
    TMC5130-BOB, TMC5130-EVAL, SilentStepStick-style TMC5130, or equivalent SPI module

Motor:
    NEMA 17 stepper, 1.5–2 A class

Power:
    24 V DC supply
    suitable motor current rating

Inputs:
    microswitch or lever switch for home
    HOME button
    MANUAL button or toggle
    STOP button optional

Wiring:
    JST/Dupont/screw terminals as convenient
```

### Mechanical

```text
Base:
    heavy plate, aluminum block, plywood block, or printed enclosure with ballast

Fixed arm:
    10-inch 2020 extrusion or aluminum side plates

Moving arm:
    10-inch 2020 extrusion, aluminum flat bar, or tube

Belt/pulleys:
    GT2 9 mm timing belt
    20T GT2 motor pulley, 5 mm bore
    60T or 80T GT2 elbow pulley
    GT2 idler pulley/tensioner

Joint:
    8 mm shaft or shoulder bolt
    2x flanged bearings
    shaft collars
    washers/spacers
    Belleville washer stack
    fiber/friction washer
    nylock nut or adjustable clamp screw

Safety:
    mechanical stop tabs
    rubber bumpers optional
```

### 9.1 Detailed Mechanical BOM, Prototype A

Pricing was checked on 2026-06-13. Prices are USD unless noted, and exclude tax and shipping. Treat vendor links and prices as procurement starting points, not locked vendors.

Design assumptions for this BOM:

```text
fixed arm length:       10 in, 2020 extrusion
moving arm length:      10 in, 2020 extrusion
drive ratio:            20T motor pulley to 80T elbow pulley, 4:1
belt profile:           2GT / GT2, 2 mm pitch, 9 mm width
elbow shaft:            8 mm
base:                   steel plate plus bolt-on ballast
fabricated parts:       drilled flat aluminum and/or 3D printed brackets
```

#### 9.1.1 Preferred Mechanical BOM

| Area | Item | Qty | Preferred source / part | Unit price | Extended | Notes |
|---|---:|---:|---|---:|---:|---|
| Base | 12 in x 12 in x 1/4 in A36 steel plate | 1 | [Amazon / IMS A36 steel plate](https://www.amazon.com/Steel-Plate-A36-Steel-25-Thick/dp/B07MC8K235) | $25.99 | $25.99 | Heavy enough for bench testing; drill for extrusion brackets and electronics standoffs. |
| Base | 5 lb cast iron weight plate for ballast | 1 | [Walmart / CAP 5 lb standard plate](https://www.walmart.com/ip/CAP-Barbell-Standard-Cast-Iron-Weight-Plate-5-Lbs-Gray/20470511) | $7.97 | $7.97 | Bolt or clamp under/behind the base if tipping is seen. |
| Arms | 20 mm x 20 mm T-slot extrusion, 10 in cut length | 2 | [TNUTZ EXM-2020](https://www.tnutz.com/product/exm-2020/) | $1.80 | $3.60 | TNUTZ lists EXM-2020 at about $0.18/in; buy two 10 in cuts or one longer length and cut locally. |
| Fabrication stock | 2 in x 36 in x 1/8 in aluminum flat bar | 1 | [Home Depot / Everbilt 6220](https://www.homedepot.com/p/Everbilt-2-in-x-3-ft-1-8-in-Thick-Aluminum-Flat-Bar-6220/332734175) | $18.98 | $18.98 | Cut elbow side plates, moving-arm hub plate, motor/tensioner plates, and stop tabs from this. |
| Extrusion hardware | 2020 inside corner bracket kit with M5 T-nuts/screws | 4 | [TNUTZ CBK-020-A kit](https://www.tnutz.com/product/cbk-020-a/) | $1.50 | $6.00 | Used for arm/base joints and quick fixture plates. |
| General metric hardware | M5 stainless button-head screw/nut/washer kit | 1 | [Home Depot / MYWISH M5 kit](https://www.homedepot.com/p/MYWISH-244-Piece-M5-Stainless-Steel-Button-Head-Hex-Socket-Cap-Screw-Assortment-Kit-with-Nuts-and-Washers-SF-TZ00004/334334771) | $11.99 | $11.99 | Covers most prototype fastening; add M3 motor screws if not included with motor bracket. |
| Motor mount | NEMA 17 aluminum motor bracket | 1 | [Pololu NEMA 17 L-bracket](https://www.pololu.com/product/2266) | $6.95 | $6.95 | Slotted bracket allows coarse belt tension adjustment at the motor. |
| Belt drive | 20T 2GT pulley, 9 mm belt, 5 mm bore | 1 | [KB3D Gates 20T 5 mm x 9 mm pulley](https://kb-3d.com/store/motion/250-gates-powergrip-2gt-pulley-20-tooth-5mm-9mm-1634481732885.html) | $4.49 | $4.49 | Matches common NEMA 17 5 mm shaft. |
| Belt drive | 80T 2GT pulley, 9 mm belt, 8 mm bore | 1 | [KB3D 80T 8 mm x 9 mm pulley](https://kb-3d.com/store/motion-system/21-2gt-pulley-16-tooth-9mm-belt-width-5mm-bore-1634485429736.html) | $12.49 | $12.49 | Elbow pulley; set-screw to 8 mm shaft. |
| Belt drive | Closed-loop 2GT belt, 9 mm width, final length TBD | 1 | [RobotDigg GT2 endless belt category](https://www.robotdigg.com/category/28) or equivalent Amazon/eBay 610-640 mm belt | $8.00-$12.00 | $12.00 | Do not buy final length until motor-to-elbow center distance is fixed. See belt-length note below. |
| Belt drive | 2GT 9 mm toothed idler pulley | 1 | [KB3D Gates 2GT 9 mm toothed idler](https://kb-3d.com/store/96-v-minion) | $5.49 | $5.49 | Optional but useful for a simple slotted tensioner. |
| Elbow shaft | 8 mm stainless shaft, 100 mm length | 1 | [ServoCity 8 mm x 100 mm shaft](https://www.servocity.com/8mm-x-100mm-stainless-steel-precision-shafting/) | $3.49 | $3.49 | Cut shorter if needed after stack-up is finalized. |
| Elbow bearings | 8 mm ID flanged ball bearing, 22 mm OD, 7 mm thick | 2 | [ServoCity 8 mm ID flanged bearing list](https://www.servocity.com/flanged-ball-bearings/?filter_bore=8mm) | $2.99 | $5.98 | One per fixed-arm side plate. F608ZZ equivalents are fine. |
| Elbow retention | 8 mm set-screw shaft collars, 2-pack | 1 | [ServoCity 8 mm set-screw collars](https://www.servocity.com/set-screw-collars/) | $4.99 | $4.99 | One collar locates the shaft; one can act as the adjustable clutch preload collar. |
| Spacers | 8 mm ID x 10 mm OD spacer/shim pack | 1 | [ServoCity 8 mm spacer related item](https://www.servocity.com/8mm-x-100mm-stainless-steel-precision-shafting/) | $2.69 | $2.69 | Use to prevent bearing drag and tune axial stack spacing. |
| Friction clutch | 2-1/2 in fiber friction washer | 2 | [American Grip fiber washer](https://americangrip.com/product/fiber-washer/) | $5.00 | $10.00 | Drill/ream center to shaft clearance if needed; one or two washers can be tested. |
| Friction clutch | M8 Belleville washer, stainless | 4 | [Accu HBW-M8-A2 Belleville washer](https://accu-components.com/us/belleville-washers/72447-HBW-M8-A2) | $1.77 | $7.08 | Stack under preload collar or nut to keep clutch force stable. |
| Hard stops | Stop tabs from aluminum flat bar | 2 | Fabricated from flat bar above | $0.00 | $0.00 | Drill slotted holes so stop angles can be adjusted. |
| Hard stops | Rubber bumper/feet assortment | 1 | Local hardware, Amazon, or McMaster | $5.00 | $5.00 | Optional but recommended at hard stops and base feet. |
| Assembly | Medium threadlocker | 1 | [KB3D Loctite 243, 6 mL](https://kb-3d.com/store/96-v-minion) | $8.69 | $8.69 | Use on pulley set screws, collar set screws, and stop hardware after adjustment. |
| Printed parts | PETG/PLA filament allowance | 1 | Existing stock or commodity filament | $5.00 | $5.00 | For belt guards, switch mounts, spacer blocks, and temporary fixtures. |

Estimated mechanical procurement total:

```text
mechanical subtotal before tax/shipping: about $169
typical tax/shipping allowance:          $25-$50
planning total:                          $195-$220
```

The practical low-cost path is to replace ServoCity/Pololu/TNUTZ parts with Amazon/eBay commodity equivalents. That can cut roughly $30-$50, but it increases fit risk and usually costs more iteration time.

#### 9.1.2 Belt Length Selection

The belt length is the main dimension that should be selected after the motor and elbow pulley centers are laid out.

For two pulleys:

```text
L ~= 2C + (pi / 2)(D + d) + ((D - d)^2 / (4C))

C = pulley center distance
D = large pulley pitch diameter
d = small pulley pitch diameter
```

For 2GT pulleys:

```text
20T pitch diameter = (20 teeth x 2 mm) / pi ~= 12.7 mm
80T pitch diameter = (80 teeth x 2 mm) / pi ~= 50.9 mm
```

Approximate closed-loop belt lengths:

```text
C = 254 mm / 10.0 in:   L ~= 610 mm
C = 260 mm:             L ~= 622 mm
C = 270 mm:             L ~= 642 mm
```

Recommendation:

```text
Buy the belt after the base/motor bracket is sketched.
Provide at least +/- 10 mm motor or idler adjustment.
If buying before CAD, buy 610 mm, 620 mm, and 640 mm commodity belts and expect one to fit.
```

#### 9.1.3 Elbow Clutch Stack For This BOM

Use the following outboard prototype stack from fixed arm outward:

```text
fixed-arm left cheek plate
8 mm flanged bearing
spacer tube between cheek plates
fixed-arm right cheek plate
8 mm flanged bearing
spacer washer
80T pulley set-screwed to 8 mm shaft
fiber friction washer
moving-arm hub plate cut from 1/8 in aluminum flat bar
Belleville washer stack
8 mm shaft collar used as adjustable preload collar
```

The moving arm hub plate should have an 8 mm clearance hole and should bolt to the moving 2020 extrusion. It should not clamp to the shaft. The 80T pulley drives the moving arm only through the friction washer and preload stack. Keep the clutch stack outside the fixed-arm cheek plates so the preload collar is accessible during tuning.

Initial clutch tuning target:

```text
tip slip force: 2-4 lbf
arm length:     10 in
slip torque:    20-40 lbf-in
```

Tune preload by hanging known weights from the moving arm tip and adjusting the outer shaft collar. After the slip point is acceptable, mark the collar position and apply threadlocker to the set screw.

---

## 10. Firmware Deliverables

Create firmware that supports:

### Required Commands / Inputs

```text
HOME button:
    start homing sequence

MANUAL button/toggle:
    enter manual mode
    disable motor or reduce hold current
    invalidate position

STOP button optional:
    immediately stop motion
    disable motor
    invalidate position
```

### Optional Serial Commands

Implement a simple serial console for testing:

```text
home
manual
enable
disable
move <degrees>
goto <degrees>
status
zero
invalidate
```

Examples:

```text
goto 45
goto 90
move -10
status
```

### Status Output

The `status` command should report:

```text
state
position_valid
current_position_steps
current_position_degrees
home_switch_state
manual_mode_state
last_fault
```

---

## 11. Calibration Constants

Firmware should have editable constants:

```text
MOTOR_STEPS_PER_REV
MICROSTEPS
MOTOR_PULLEY_TEETH
ELBOW_PULLEY_TEETH
ELBOW_MIN_DEG
ELBOW_MAX_DEG
HOME_DIRECTION
HOME_SPEED
MOVE_SPEED
MOVE_ACCEL
HOMING_TIMEOUT_STEPS
```

For 20T to 80T pulley:

```text
belt_ratio = 80 / 20 = 4.0
```

For a 200-step motor at 16 microsteps:

```text
motor_steps_per_rev = 200 × 16 = 3200
elbow_steps_per_rev = 3200 × 4 = 12800
steps_per_degree = 12800 / 360 ≈ 35.56
```

---

## 12. Test Plan

### Mechanical Tests

1. Verify fixed arm stiffness.
2. Verify moving arm rotates freely on elbow bearings.
3. Verify timing belt alignment.
4. Verify belt tension adjustment.
5. Verify elbow pulley rotation drives moving arm through friction hub.
6. Verify friction hub slips before structure bends.
7. Verify hard stops work.
8. Verify home switch actuates before hard stop.

### Electrical Tests

1. Confirm MCU can configure TMC5130 over SPI.
2. Confirm motor moves both directions.
3. Confirm motor current settings are safe.
4. Confirm home switch reads correctly.
5. Confirm buttons read correctly.
6. Confirm STOP or disable behavior works.

### Firmware Tests

1. Power-on enters invalid/unhomed state.
2. Robotic moves are rejected before homing.
3. HOME button starts homing.
4. Homing sets zero and enables robotic mode.
5. `goto` commands move to valid positions.
6. Commands outside travel range are rejected.
7. MANUAL mode disables/reduces motor current and invalidates position.
8. After MANUAL, moves are rejected until HOME.
9. Homing timeout enters fault state.
10. STOP enters fault/invalid state.

### Slip/Recovery Tests

1. Home the arm.
2. Command a known angle.
3. Force the moving arm until friction hub slips.
4. Confirm nothing breaks.
5. Press HOME.
6. Confirm system re-homes and resumes.

---

## 13. Success Criteria

The prototype is successful if:

```text
The base-mounted motor can drive the remote elbow joint through the belt.
The moving 10-inch arm can carry a 250–500 g dummy load.
The arm can move smoothly through approximately 0–135°.
The system homes repeatably using only a contact switch.
The friction hub protects the joint during manual hijack.
After manual hijack, robotics remains disabled until HOME is pressed.
No encoder or StallGuard is required for correct basic operation.
The BOM is mostly off-the-shelf.
```

---

## 14. Build Philosophy

This is a procurement-first prototype.

Every part should be one of:

```text
off-the-shelf
3D printed
cut/drilled from flat stock
assembled from standard hardware
```

Avoid:

```text
custom PCB
custom machined pulley
precision gearbox
worm gear
complex clutch
encoder dependency
vision system
full robot arm
```

The goal is to learn quickly whether a remotely driven, back-drivable, homed, single-joint inspection-light arm is mechanically and behaviorally viable.
