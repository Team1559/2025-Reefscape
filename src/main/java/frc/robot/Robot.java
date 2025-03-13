// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        private static final double SWERVE_MAX_ANGULAR_VELOCITY = 9;
        private static final double SWERVE_SLOW_LINEAR_VELOCITY = SWERVE_MAX_LINEAR_VELOCITY / 4;
        private static final double SWERVE_SLOW_ANGULAR_VELOCITY = SWERVE_MAX_ANGULAR_VELOCITY / 4;
        private final Climber2025 climber;

        // private final PowerDistribution pdh;
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
                // pdh = new PowerDistribution(0, ModuleType.kRev);
                // pdh.setSwitchableChannel(true);
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

                // NamedCommands.registerCommand("elevatorL1", elevatorL1);
                // NamedCommands.registerCommand("elevatorL2", elevatorL2);
                // NamedCommands.registerCommand("elevatorL3", elevatorL3);
                // NamedCommands.registerCommand("elevatorL4", elevatorL4);
                // NamedCommands.registerCommand("elevatorFeeder", elevatorFeeder);

                Command coralAngleFeeder = new CoralIntakeAngleCommand(coralIntake,
                                CoralIntake.TargetAngle.SOURCE_ANGLE);
                Command coralAngleL2 = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L2_ANGLE);
                Command coralAngleL3 = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L2_ANGLE);
                Command coralAngleL4 = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L4_ANGLE);
                Command coralAngleStow = new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.STOW);

                // NamedCommands.registerCommand("coralAngleFeeder", coralAngleFeeder);
                // NamedCommands.registerCommand("coralAngleL2", coralAngleL2);
                // NamedCommands.registerCommand("coralAngleL3", coralAngleL3);
                // NamedCommands.registerCommand("coralAngleL4", coralAngleL4);

                Command algaeLevelTwo = new ElevatorHeightCommand2025(elevator, Level.L2_ALGAE);
                Command algaeLevelThree = new ElevatorHeightCommand2025(elevator, Level.L3_ALGAE);
                Command algaeDown = new ElevatorHeightCommand2025(elevator, Level.FEEDER);

                NamedCommands.registerCommand("algaeL2", algaeLevelTwo);
                NamedCommands.registerCommand("algaeL3", algaeLevelThree);
                NamedCommands.registerCommand("algaeDown", algaeDown);

                Supplier<Command> algaeStow = () -> new AlgaeIntakeAngleCommand(algaeIntake,
                                AlgaeIntake.TargetAngle.STOWED);
                Command algaeReef = new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.REEF);
                Command algaeFloor = new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.FLOOR);

                NamedCommands.registerCommand("algaeStow", algaeStow.get());
                NamedCommands.registerCommand("algaeReef", algaeReef);
                NamedCommands.registerCommand("algaeFloor", algaeFloor);

                Command coralAlignFeeder = new SequentialCommandGroup(algaeStow.get(),
                                elevatorFeeder, coralAngleFeeder);
                Command coralAlignL2 = new SequentialCommandGroup(algaeStow.get(), elevatorL2,
                                coralAngleL2);
                Command coralAlignL3 = new SequentialCommandGroup(algaeStow.get(), elevatorL3,
                                coralAngleL3);
                Command coralAlignL4 = new SequentialCommandGroup(algaeStow.get(), elevatorL4,
                                coralAngleL4);

                NamedCommands.registerCommand("coralAlignFeeder", coralAlignFeeder);
                NamedCommands.registerCommand("coralAlignL2", coralAlignL2);
                NamedCommands.registerCommand("coralAlignL3", coralAlignL3);
                NamedCommands.registerCommand("coralAlignL4", coralAlignL4);

                Command climb = new StartEndCommand(climber::run, climber::stop, climber);
                NamedCommands.registerCommand("climb", climb);

                Command coralIn = new StartEndCommand(() -> coralIntake.run(true), () -> coralIntake.stop(),
                                coralIntake);
                Command coralOut = new StartEndCommand(() -> coralIntake.run(false), () -> coralIntake.stop(),
                                coralIntake);
                NamedCommands.registerCommand("coralIn", coralIn);
                NamedCommands.registerCommand("coralOut", coralOut);

                Command algaeIn = new StartEndCommand(() -> algaeIntake.run(false), () -> algaeIntake.stop(),
                                algaeIntake);
                Command algaeOut = new StartEndCommand(() -> algaeIntake.run(true), () -> algaeIntake.stop(),
                                algaeIntake);
                NamedCommands.registerCommand("algaeIn", algaeIn);
                NamedCommands.registerCommand("algaeOut", algaeOut);

                Command elevatorHome = new InstantCommand(
                                () -> elevator.goHome(),
                                elevator);
                NamedCommands.registerCommand("elevatorHome", elevatorHome);

                Command manualElevatorUp = new InstantCommand(
                                () -> elevator.changeTargetPosition(.005), elevator);
                NamedCommands.registerCommand("manualElevatorUp", manualElevatorUp);
        }

        public void setTeleopBindings() {
                Trigger algaeMod = coPilotController.leftTrigger();
                Trigger robotOrientedMod = pilotController.leftTrigger();
                drivetrain.setDefaultCommand(
                                new TeleopDriveCommand(() -> -pilotController.getLeftY(),
                                                () -> -pilotController.getLeftX(),
                                                () -> -pilotController.getRightX(), SWERVE_MAX_LINEAR_VELOCITY,
                                                SWERVE_MAX_ANGULAR_VELOCITY,
                                                drivetrain, robotOrientedMod));
                pilotController.leftBumper().whileTrue(new TeleopDriveCommand(() -> -pilotController.getLeftY(),
                                () -> -pilotController.getLeftX(),
                                () -> -pilotController.getRightX(), SWERVE_SLOW_LINEAR_VELOCITY,
                                SWERVE_SLOW_ANGULAR_VELOCITY,
                                drivetrain, robotOrientedMod));
                pilotController.rightTrigger().whileTrue(NamedCommands.getCommand("climb"));

                coPilotController.a().and(algaeMod.negate()).onTrue(NamedCommands.getCommand("coralAlignFeeder"));
                coPilotController.b().and(algaeMod.negate()).onTrue(NamedCommands.getCommand("coralAlignL2"));
                coPilotController.x().and(algaeMod.negate()).onTrue(NamedCommands.getCommand("coralAlignL3"));
                coPilotController.y().and(algaeMod.negate()).onTrue(NamedCommands.getCommand("coralAlignL4"));

                coPilotController.a().and(algaeMod).onTrue(NamedCommands.getCommand("algaeDown"));
                coPilotController.b().and(algaeMod).onTrue(NamedCommands.getCommand("algaeL2"));
                coPilotController.x().and(algaeMod).onTrue(NamedCommands.getCommand("algaeL3"));

                coPilotController.rightTrigger().and(algaeMod.negate()).whileTrue(NamedCommands.getCommand("coralIn"));
                coPilotController.rightBumper().and(algaeMod.negate()).whileTrue(NamedCommands.getCommand("coralOut"));

                coPilotController.rightTrigger().and(algaeMod).whileTrue(NamedCommands.getCommand("algaeIn"));
                coPilotController.rightBumper().and(algaeMod).whileTrue(NamedCommands.getCommand("algaeOut"));

                coPilotController.povUp().onTrue(NamedCommands.getCommand("algaeStow"));
                coPilotController.povDown().onTrue(NamedCommands.getCommand("algaeFloor"));
                coPilotController.povRight().onTrue(NamedCommands.getCommand("algaeReef"));
        }

        public void setTestBindings() {
                Trigger robotOrientedMod = pilotController.leftTrigger();
                // coPilotController.a().onTrue(NamedCommands.getCommand("elevatorL1"));
                // coPilotController.b().and(coPilotController.rightTrigger().negate())
                //                 .onTrue(NamedCommands.getCommand("elevatorL2"));
                // coPilotController.b().and(coPilotController.rightTrigger()).onTrue(NamedCommands.getCommand("algaeL2"));
                // coPilotController.x().and(coPilotController.rightTrigger().negate())
                //                 .onTrue(NamedCommands.getCommand(("elevatorL3")));
                // coPilotController.x().and(coPilotController.rightTrigger())
                //                 .onTrue(NamedCommands.getCommand("elevatorL4"));

                // coPilotController.y().onTrue(NamedCommands.getCommand("elevatorL4"));
                // coPilotController.start().onTrue(NamedCommands.getCommand("manualElevatorUp"));// .onTrue(elevatorHome);

                drivetrain.setDefaultCommand(
                                new TeleopDriveCommand(() -> pilotController.getLeftY(),
                                                () -> pilotController.getLeftX(),
                                                () -> -pilotController.getRightX(), SWERVE_MAX_LINEAR_VELOCITY,
                                                SWERVE_MAX_ANGULAR_VELOCITY,
                                                drivetrain, robotOrientedMod));
                pilotController.leftBumper().whileTrue(new TeleopDriveCommand(() -> -pilotController.getLeftY(),
                                () -> -pilotController.getLeftX(),
                                () -> -pilotController.getRightX(), SWERVE_SLOW_LINEAR_VELOCITY,
                                SWERVE_SLOW_ANGULAR_VELOCITY,
                                drivetrain, robotOrientedMod));
                // coPilotController.rightTrigger()
                //                 .whileTrue(new StartEndCommand(() -> algaeIntake.run(false), () -> algaeIntake.stop(),
                //                                 algaeIntake));
                // coPilotController.rightBumper()
                //                 .whileTrue(new StartEndCommand(() -> algaeIntake.run(true), () -> algaeIntake.stop(),
                //                                 algaeIntake));

                // coPilotController.leftTrigger()
                //                 .whileTrue(new StartEndCommand(() -> coralIntake.run(false), () -> coralIntake.stop(),
                //                                 coralIntake));
                // coPilotController.leftBumper()
                //                 .whileTrue(new StartEndCommand(() -> coralIntake.run(true), () -> coralIntake.stop(),
                //                                 coralIntake));

                // coPilotController.povUp()
                //                 .onTrue(new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.PROCESSOR));
                // coPilotController.povDown().onTrue(new AlgaeIntakeAngleCommand(algaeIntake,
                //                 AlgaeIntake.TargetAngle.FLOOR));

                // coPilotController.povLeft().onTrue(new CoralIntakeAngleCommand(coralIntake,
                //                 CoralIntake.TargetAngle.BARGE));
                // coPilotController.povRight().onTrue(new CoralIntakeAngleCommand(coralIntake,
                //                 CoralIntake.TargetAngle.L4_ANGLE));
                // // coPilotController.povUp().onTrue(new CoralIntakeAngleCommand(coralIntake,
                // // CoralIntake.TargetAngle.SOURCE_ANGLE));

                pilotController.rightTrigger().whileTrue(NamedCommands.getCommand("climb"));
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
