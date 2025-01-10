#### Back to [README](/README.md)

# Game Commands

## Controllers
* **Pilot Controller:** `[P]`
* **Co-Pilot Controller:** `[CP]`

## Default Commands
These Commands will run, unless another command is called on the subsystem(s).

* **LEDs: Team Color**
    * The LEDs will display the _Team Color_
* **Stop Flywheel**
    * The Flywheel will be stopped, if it has not been given a command in over 2 seconds
        * (For safety)

## Triggers
These are Commands that are triggered by things other then the controllers.

* **LEDs: Has Note**
    * While the Color Sensor detects a Note, the LEDs will be $\color{orange}Orange$
* **Thermal Cutoffs**
    * Any Motor exceeding their recommended temperature range, will be disabled until its temperature returns to a safe range
        * (For safety)
    * LEDs will flash $\color{yellow}Yellow$ while the motor is disabled

<br/>

# Controller Commands
These Commands are triggered by the controllers.

## Driving Commands
* **Drive Forward / Backwards** `[P: Left Joystick Y]`
* **Drive Left / Right** `[P: Left Joystick X]`
* **Spin Clockwise / Counterclockwise** `[P: Right Joystick X]`
* **Aim at Speaker** `[P: Left Trigger]`
    * The robot will maintain an aim at the Speaker
        * (This will disable the Spin Command)
    * The Flywheel will spin up
    * While Aiming, the LEDs will flash the _Team Color_
    * When in range, the LEDs will turn $\color{green}Green$
* **Aim at Amp** `[P: Right Trigger]`
    * The robot will maintain an aim at the Amp
        * (This will disable the Spin Command)
    * The Aimer will automatically set it's angle based on the distance to target
    * The Flywheel will spin up
    * While Aiming, the LEDs will flash the _Team Color_
    * When in range, the LEDs will turn $\color{green}Green$

## Intake Commands
Since the Intake operates automatically as a default command, these commands are only for abnormal situations.
* **Run Intake & Feeder** `[CP: Left Trigger]`
    * The Intake and Feeder will stop, when the Color Sensor detects a Note
    * (The Flywheel will be stopped, if running, so that the note is not accidentally shot)
* **Reverse Intake & Feeder** `[CP: X]`
    * (The Flywheel will be stopped, if running)

## Shooter Commands
* **Shoot** `[CP: Right Trigger]`
    * The Flywheel will spin up
        * (If not already spinning)
    * The Feeder will move the note to the Flywheel
    * LEDs will flash $\color{violet}Purple$
        * Once the Color Sensor no longer sees the Note
    * The Flywheel will stop
* **Stop Flywheel & Feeder** `[CP: B]`
    * (The Intake will also be stopped, if running)
* **Reverse Flywheel & Feeder** `[CP: A]`
    * (The Intake will be stopped, if running)

## Climber Commands
* **Climber Extend** `[CP: D-Pad Up]`
* **Climber Retract** `[CP: D-Pad Down]`
* **Traverse Left / Right** `[CP: D-Pad Left/Right]`

## Aimer Commands
**Note:** The Aimer will aim automatically, these commands are just for abnormal situations.

* **Aim Up / Down** `[CP: Right Joystick Y]`
