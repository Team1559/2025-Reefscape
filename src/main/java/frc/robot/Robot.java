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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.commands.ElevatorHeightCommand2025;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator2025;
import frc.robot.subsystems.Elevator2025.IntakeOffset;
import frc.robot.subsystems.Elevator2025.Level;
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
        Command levelOne = new ElevatorHeightCommand2025(elevator, Level.L1, IntakeOffset.CORAL);
        Command levelTwo = new ElevatorHeightCommand2025(elevator, Level.L2, IntakeOffset.CORAL);
        Command levelThree = new ElevatorHeightCommand2025(elevator, Level.L3, IntakeOffset.CORAL);
        Command levelFour = new ElevatorHeightCommand2025(elevator, Level.L4, IntakeOffset.CORAL);
        Command reset = new InstantCommand(
                () -> elevator.goHome(),
                elevator);

        pilotController.a().onTrue(levelOne);
        pilotController.b().onTrue(levelTwo);
        pilotController.x().onTrue(levelThree);
        pilotController.y().onTrue(levelFour);
        pilotController.povDown().onTrue(reset);

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
        FunctionalCommand goToZero = new FunctionalCommand(elevator::goHome, () -> {
        }, (b) -> {
        }, elevator::isHome, elevator);
        CommandScheduler.getInstance().schedule(goToZero);

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
