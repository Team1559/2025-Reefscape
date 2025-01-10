#### Back to [README](/README.md)

# Technician Documentation
## Technician Controller
* **Driving:**
    * `[D-Pad]` Provides isolated directional commands
* **LEDs:** (On Hold)
    * `[A + Back]` = Solid $\color{green}Green$
    * `[B + Back]` = Solid $\color{red}Red$ and **Black** pattern
    * `[X + Back]` = $\color{blue}Blue$ and $\color{purple}Purple$ pattern moving in one direction
    * `[Y + Back]` = $\color{yellow}Yellow$ and **Black** pattern moving in the other direction
    * `[Left Bumber + Back]` = <span style="font-size:smaller;">Decrease Brightness</span>
    * `[Right Bumber + Back]` = <span style="font-size:larger;">Increase Brightness</span>
    * `[Back + Start]` = Turn Off LEDs
* **Subsystems:**
    * `[A]` Runs the Intake
    * `[A + Start]` Runs the Intake in Reverse
    * `[B]` Runs the Feeder
    * `[B + Start]` Runs the Feeder in Reverse
    * `[Y]` Runs the Flywheel
    * `[Y + Start]` Runs the Flywheel in Reverse
    * `[Left Bumper + Start]` Runs just the left Flywheel motor
    * `[Right Bumper + Start]` Runs just the left Flywheel motor
    * `[Left Bumper]` Moves the Aimer DOWN 5 degrees
    * `[Right Bumper]` Moves the Aimer UP 5 degrees

## LED States

| Color  | State |
---------|--------
| Red    | Normal operation for Red alliance robot  |
| Blue   | Normal operation for Blue alliance robot |
| Orange | Note in the system; intake off.          |
| Violet | Single blink when shooting completes.    |
| Green  | Intake disabled by copilot.              |