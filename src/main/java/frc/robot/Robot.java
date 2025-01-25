// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystems.Leds;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;

public class Robot extends LoggedRobot {
    private final CommandXboxController pilotController;
    // private final CommandXboxController coPilotController;

    private final Drivetrain drivetrain;
    private final Leds led;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg

        pilotController = new CommandXboxController(0);
        // coPilotController = new CommandXboxController(1);

        drivetrain = new Drivetrain();
        led = new Leds(0, 144);
    }

    @Override
    public void robotInit() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, 6, 6, drivetrain));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        LEDPattern pattern = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2));
        led.setPattern(pattern);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
        LEDPattern rainbow = LEDPattern.rainbow(255,250);
        LEDPattern pattern=rainbow.scrollAtRelativeSpeed(Percent.per(Second).of(50));
        led.setPattern(pattern);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
