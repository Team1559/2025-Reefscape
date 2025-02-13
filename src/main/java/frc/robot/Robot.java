// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator2025;
import frc.robot.subsystems.Vision2025;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    // private final CommandXboxController coPilotController;

    private final Drivetrain drivetrain;
    private final Vision2025 vision;
    private final Elevator2025 elevator;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg

        pilotController = new CommandXboxController(0);
        // coPilotController = new CommandXboxController(1);

        drivetrain = new Drivetrain();
        vision = new Vision2025(drivetrain);
        drivetrain.configureAuto(23.2, 8);
        elevator = new Elevator2025();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void robotInit() {
        double elevatorOffset = Units.inchesToMeters(35); // TODO: offset between the target height from ground and
                                                          // elevator target height
        InstantCommand levelOne = new InstantCommand(
                () -> elevator.setTargetPosition(Units.inchesToMeters(18) - elevatorOffset),
                elevator); // TODO: change heights to real height after starting height is accounted for
        InstantCommand levelTwo = new InstantCommand(
                () -> elevator.setTargetPosition(Units.inchesToMeters(31.875) - elevatorOffset),
                elevator);
        InstantCommand levelThree = new InstantCommand(
                () -> elevator.setTargetPosition(Units.inchesToMeters(42.625) - elevatorOffset),
                elevator);
        InstantCommand levelFour = new InstantCommand(
                () -> elevator.setTargetPosition(Units.inchesToMeters(72) - elevatorOffset),
                elevator);

        pilotController.a().onTrue(levelOne);
        pilotController.b().onTrue(levelTwo);
        pilotController.x().onTrue(levelThree);
        pilotController.y().onTrue(levelFour);
        
        drivetrain.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, 5.21, 1.925, drivetrain));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(autoChooser.getSelected());
    }

    @Override
    public void teleopInit() {
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
