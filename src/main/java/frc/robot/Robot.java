// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.tools.JavaCompiler.CompilationTask;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.commands.IntakeAngleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator2025;
import frc.robot.subsystems.Vision2025;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.Intake;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;

    private final Drivetrain drivetrain;
    private final Vision2025 vision;
    private final Elevator2025 elevator;
    private final Intake coralIntake;
    private final Intake algaeIntake;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg

        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);

        drivetrain = new Drivetrain();
        vision = new Vision2025(drivetrain);
        drivetrain.configureAuto(23.2, 8);
        elevator = new Elevator2025();
        coralIntake = new CoralIntake();
        algaeIntake = new AlgaeIntake();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    @Override
    public void robotInit() {
        drivetrain.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, 5.21, 1.925, drivetrain));

        coPilotController.rightTrigger().onTrue(new InstantCommand(() -> algaeIntake.run(false), algaeIntake));
        coPilotController.rightBumper().onTrue(new InstantCommand(() -> algaeIntake.run(true), algaeIntake));
        coPilotController.rightBumper().or(coPilotController.rightTrigger())
                .onFalse(new InstantCommand(algaeIntake::stop, algaeIntake));

        coPilotController.leftTrigger().onTrue(new InstantCommand(() -> coralIntake.run(false), coralIntake));
        coPilotController.leftBumper().onTrue(new InstantCommand(() -> coralIntake.run(true), coralIntake));
        coPilotController.leftBumper().or(coPilotController.leftTrigger())
                .onFalse(new InstantCommand(coralIntake::stop, coralIntake));

        coPilotController.povUp().onTrue(new IntakeAngleCommand(algaeIntake, new Rotation2d(), Rotation2d.fromDegrees(3)));
        coPilotController.povDown().onTrue(new IntakeAngleCommand(algaeIntake, new Rotation2d(), Rotation2d.fromDegrees(3)));
        
        coPilotController.povLeft().onTrue(new IntakeAngleCommand(coralIntake, new Rotation2d(), Rotation2d.fromDegrees(3)));
        coPilotController.povRight().onTrue(new IntakeAngleCommand(coralIntake, new Rotation2d(), Rotation2d.fromDegrees(3)));
        //TODO: ADD ANGLES FOR ALL ROTATION2D'S. PR (Pull Request)
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
