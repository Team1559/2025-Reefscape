// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.commands.AlgaeIntakeAngleCommand;
import frc.robot.commands.CoralIntakeAngleCommand;
import frc.robot.commands.ElevatorHeightCommand2025;
import frc.robot.subsystems.Elevator2025;
import frc.robot.subsystems.Elevator2025.Level;
import frc.robot.subsystems.SwerveDrive2025;
import frc.robot.subsystems.Vision2025;
import frc.robot.subsystems.climber.Climber2025;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.CoralIntake;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;

    private final SwerveDrive2025 drivetrain;
    private final Vision2025 vision;
    private final Elevator2025 elevator;
    private final CoralIntake coralIntake;
    private final AlgaeIntake algaeIntake;

    private static final double SWERVE_MAX_LINEAR_VELOCITY = 5.21;
    private static final double SWERVE_MAX_ANGULAR_VELOCITY = 1.925;
    private final Climber2025 climber;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg

        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);

        drivetrain = new SwerveDrive2025();

        vision = new Vision2025(drivetrain);
        elevator = new Elevator2025();
        coralIntake = new CoralIntake();
        algaeIntake = new AlgaeIntake();
        climber = new Climber2025();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void clearCommandBindings() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    public void makeCommands() {
        Command elevatorL1 = new ElevatorHeightCommand2025(elevator, Level.L1_CORAL);
        Command elevatorL2 = new ElevatorHeightCommand2025(elevator, Level.L2_CORAL);
        Command elevatorL3 = new ElevatorHeightCommand2025(elevator, Level.L3_CORAL);
        Command elevatorL4 = new ElevatorHeightCommand2025(elevator, Level.L4_CORAL);
        Command elevatorFeeder = new ElevatorHeightCommand2025(elevator, Level.FEEDER);

        NamedCommands.registerCommand("elevatorL1", elevatorL1);
        NamedCommands.registerCommand("elevatorL2", elevatorL2);
        NamedCommands.registerCommand("elevatorL3", elevatorL3);
        NamedCommands.registerCommand("elevatorL4", elevatorL4);
        NamedCommands.registerCommand("elevatorFeeder", elevatorFeeder);

        Command coralAngleFeeder = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.SOURCE_ANGLE);
        Command coralAngleL2 = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L2_ANGLE);
        Command coralAngleL3 = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L2_ANGLE);
        Command coralAngleL4 = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L2_ANGLE);

        NamedCommands.registerCommand("coralAngleFeeder", coralAngleFeeder);
        NamedCommands.registerCommand("coralAngleL2", coralAngleL2);
        NamedCommands.registerCommand("coralAngleL3", coralAngleL3);
        NamedCommands.registerCommand("coralAngleL4", coralAngleL4);

        Command algaeLevelTwo = new ElevatorHeightCommand2025(elevator, Level.L2_ALGAE);
        Command algaeLevelThree = new ElevatorHeightCommand2025(elevator, Level.L3_ALGAE);

        NamedCommands.registerCommand("algaeLevel2", algaeLevelTwo);
        NamedCommands.registerCommand("algaeLevel3", algaeLevelThree);

        Command stowAlgae = new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.STOWED);

        // Command coralAlignFeeder = new SequentialCommandGroup(stowAlgae,
        // elevatorFeeder, coralAngleFeeder);
        // Command coralAlignL2 = new SequentialCommandGroup(stowAlgae, elevatorL2,
        // coralAngleL2);
        // Command coralAlignL3 = new SequentialCommandGroup(stowAlgae, elevatorL3,
        // coralAngleL3);
        // Command coralAlignL4 = new SequentialCommandGroup(stowAlgae, elevatorL4,
        // coralAngleL4);

        // NamedCommands.registerCommand("coralAlignL2", coralAlignL2);
        // NamedCommands.registerCommand("coralAlignL3", coralAlignL3);
        // NamedCommands.registerCommand("coralAlignL4", coralAlignL4);

        Command coralIn = new StartEndCommand(() -> coralIntake.run(false), () -> coralIntake.stop(), coralIntake);
        Command coralOut = new StartEndCommand(() -> coralIntake.run(true), () -> coralIntake.stop(), coralIntake);

        Command climb = new StartEndCommand(climber::run, climber::stop, climber);
        NamedCommands.registerCommand("climb", climb);
        // TODO: verify in/out
        NamedCommands.registerCommand("coralIn", coralIn);
        NamedCommands.registerCommand("coralOut", coralOut);

        Command elevatorHome = new InstantCommand(
                () -> elevator.goHome(),
                elevator);
        NamedCommands.registerCommand("elevatorHome", elevatorHome);

        Command manualElevatorUp = new InstantCommand(
                () -> elevator.changeTargetPosition(.005), elevator);
        NamedCommands.registerCommand("manualElevatorUp", manualElevatorUp);
    }

    public void setTeleopBindings() {
        drivetrain.setDefaultCommand(
                new TeleopDriveCommand(() -> -pilotController.getLeftY(), () -> -pilotController.getLeftX(),
                        () -> -pilotController.getRightX(), SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_ANGULAR_VELOCITY,
                        drivetrain));
        pilotController.rightTrigger().whileTrue(NamedCommands.getCommand("climb"));

        coPilotController.a().onTrue(NamedCommands.getCommand("coralAlignFeeder"));
        coPilotController.b().onTrue(NamedCommands.getCommand("coralAlignL2"));
        coPilotController.x().onTrue(NamedCommands.getCommand("coralAlignL3"));
        coPilotController.y().onTrue(NamedCommands.getCommand("coralAlignL4"));

        coPilotController.rightBumper().whileTrue(NamedCommands.getCommand("coralIn"));
        coPilotController.rightTrigger().whileTrue(NamedCommands.getCommand("coralOut"));

    }

    public void setTestBindings() {
        coPilotController.a().onTrue(NamedCommands.getCommand("elevatorL1"));
        coPilotController.b().and(coPilotController.rightTrigger().negate())
                .onTrue(NamedCommands.getCommand("elevatorL2"));
        coPilotController.b().and(coPilotController.rightTrigger()).onTrue(NamedCommands.getCommand("algaeLevel2"));
        coPilotController.x().and(coPilotController.rightTrigger().negate())
                .onTrue(NamedCommands.getCommand(("elevatorL3")));
        coPilotController.x().and(coPilotController.rightTrigger()).onTrue(NamedCommands.getCommand("elevatorL4"));

        coPilotController.y().onTrue(NamedCommands.getCommand("elevatorL4"));
        coPilotController.start().onTrue(NamedCommands.getCommand("manualElevatorUp"));// .onTrue(elevatorHome);

        drivetrain.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_ANGULAR_VELOCITY, drivetrain));
        pilotController.a().onTrue(new InstantCommand(elevator::stop));// TODO: temp

        coPilotController.rightTrigger()
                .whileTrue(new StartEndCommand(() -> algaeIntake.run(false), () -> algaeIntake.stop(), algaeIntake));
        coPilotController.rightBumper()
                .whileTrue(new StartEndCommand(() -> algaeIntake.run(true), () -> algaeIntake.stop(), algaeIntake));

        coPilotController.leftTrigger()
                .whileTrue(new StartEndCommand(() -> coralIntake.run(false), () -> coralIntake.stop(), coralIntake));
        coPilotController.leftBumper()
                .whileTrue(new StartEndCommand(() -> coralIntake.run(true), () -> coralIntake.stop(), coralIntake));

        coPilotController.povUp().onTrue(new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.STOWED));
        // coPilotController.povDown().onTrue(new AlgaeIntakeAngleCommand(algaeIntake,
        // AlgaeIntake.TargetAngle.REEF));

        coPilotController.povLeft().onTrue(new CoralIntakeAngleCommand(coralIntake,
                CoralIntake.TargetAngle.L2_ANGLE));
        coPilotController.povRight().onTrue(new CoralIntakeAngleCommand(coralIntake,
                CoralIntake.TargetAngle.L4_ANGLE));
        coPilotController.povUp().onTrue(new CoralIntakeAngleCommand(coralIntake,
                CoralIntake.TargetAngle.SOURCE_ANGLE));

        pilotController.rightTrigger().whileTrue(NamedCommands.getCommand("climb"));
        pilotController.leftTrigger().whileTrue(new StartEndCommand(climber::testReverse, climber::stop, climber));
    }

    @Override
    public void robotInit() {
        makeCommands();
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

        clearCommandBindings();
        setTeleopBindings();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        FunctionalCommand goToZero = new FunctionalCommand(elevator::goHome, () -> {
        }, (b) -> {
        }, elevator::isHome, elevator);
        CommandScheduler.getInstance().schedule(goToZero);
        clearCommandBindings();
        setTestBindings();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
