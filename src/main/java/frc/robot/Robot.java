// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.opencv.core.Point;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.Leds;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.commands.CoralIntakeAngleCommand;
import frc.robot.commands.ElevatorHeightCommand2025;
import frc.robot.subsystems.Elevator2025;
import frc.robot.subsystems.Elevator2025.Level;
import frc.robot.subsystems.SwerveDrive2025;
import frc.robot.subsystems.Vision2025;
import frc.robot.subsystems.climber.Climber2025;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.CoralIntake.TargetAngle;

public class Robot extends LoggedRobot {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController pilotController;
    private final CommandXboxController coPilotController;

    private final SwerveDrive2025 drivetrain;
    private final Vision2025 vision;
    private final Elevator2025 elevator;
    private final CoralIntake coralIntake;
    private final Climber2025 climber;
    private final Leds leds;

    private static final double SWERVE_MAX_LINEAR_VELOCITY = 5.21;
    private static final double SWERVE_MAX_ANGULAR_VELOCITY = 18;
    private static final double SWERVE_SLOW_LINEAR_VELOCITY = SWERVE_MAX_LINEAR_VELOCITY / 6;
    private static final double SWERVE_SLOW_ANGULAR_VELOCITY = SWERVE_MAX_ANGULAR_VELOCITY / 6;

    private static final double SWERVE_MAX_ANGULAR_ACCEL = SWERVE_MAX_ANGULAR_VELOCITY / .01;

    public Robot() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); // Leave as easter egg

        pilotController = new CommandXboxController(0);
        coPilotController = new CommandXboxController(1);

        elevator = new Elevator2025();
        drivetrain = new SwerveDrive2025();
        vision = new Vision2025(drivetrain);
        coralIntake = new CoralIntake();
        climber = new Climber2025();
        leds = new Leds(0, 68);

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("a");
        autoChooser.addOption("Pathplannerless Leave", autoDrive());
        autoChooser.addOption("Pathplannerless L4", autoL4());
        SmartDashboard.putData(autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);

        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
        pdh.setSwitchableChannel(true);
    }

    public void clearCommandBindings() {
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    public void registerNamedCommands() {
        NamedCommands.registerCommand("print", new PrintCommand("printed"));
        NamedCommands.registerCommand("alignL4", coralAlignL4());
        NamedCommands.registerCommand("alignL3", coralAlignL3());
        NamedCommands.registerCommand("coralOutAuto", coralOutSlow());
        NamedCommands.registerCommand("alignFeeder", coralAlignFeeder());
        NamedCommands.registerCommand("coralIn", coralIn());
    }

    public Command limitAccel(Level level, boolean slow) {
        return new InstantCommand(() -> drivetrain.setAccelerationLimits(
                50 * (slow ? 2 : 1) * (1 - (level.height / Level.L4_CORAL.height) * .25),
                Rotation2d.fromRadians(SWERVE_MAX_ANGULAR_ACCEL)));
    }

    public Command elevatorTo(Level level) {
        return new ElevatorHeightCommand2025(elevator, level);
    }

    public Command coralAngleFeeder(TargetAngle targetAngle) {
        return new CoralIntakeAngleCommand(coralIntake, targetAngle);
    }

    public Command coralAlignFeeder() {
        return elevatorTo(Level.FEEDER).alongWith(coralAngleFeeder(TargetAngle.SOURCE_ANGLE));
    }

    public Command coralAlignL1() {
        return elevatorTo(Level.L1_CORAL).alongWith(coralAngleFeeder(TargetAngle.L1_ANGLE));
    }

    public Command coralAlignL2() {
        return elevatorTo(Level.L2_CORAL).alongWith(coralAngleFeeder(TargetAngle.L2_ANGLE));
    }

    public Command coralAlignL3() {
        return elevatorTo(Level.L3_CORAL).alongWith(coralAngleFeeder(TargetAngle.L3_ANGLE));
    }

    public Command coralAlignL4() {
        return elevatorTo(Level.L4_CORAL).alongWith(coralAngleFeeder(TargetAngle.L4_ANGLE));
    }

    public Command climb() {
        return new StartEndCommand(climber::run, climber::stop, climber);
    }

    public Command coralIn() {
        return new StartEndCommand(() -> coralIntake.run(true), () -> coralIntake.stop(), coralIntake);
    }

    public Command coralOut() {
        return new StartEndCommand(() -> coralIntake.run(false), () -> coralIntake.stop(),
                coralIntake);
    }

    public Command coralOutSlow() {
        return new StartEndCommand(() -> coralIntake.run(false, true), () -> coralIntake.stop(), coralIntake);
    }

    public Command elevatorHome() {
        return new InstantCommand(() -> elevator.goHome(), elevator);
    }

    public Command manualElevatorUp() {
        return new InstantCommand(() -> elevator.changeTargetPosition(.005));
    }

    public Command autoDrive() {
        return new TeleopDriveCommand(
                () -> .25,
                () -> 0, () -> 0, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_ANGULAR_VELOCITY, drivetrain,
                () -> true).finallyDo((x) -> drivetrain.stopDriving());
    }

    public Command autoL4() {
        return autoDrive().alongWith(coralAlignL4()).withTimeout(2).andThen(new WaitCommand(1.5))
                .andThen(coralOutSlow().withTimeout(1));
    }

    public void setTeleopBindings() {
        // Pilot modifier buttons
        Trigger robotOrientedMod = pilotController.leftTrigger();
        Trigger slowMod = pilotController.rightTrigger();

        // Pilot controls
        drivetrain.setDefaultCommand(
                new TeleopDriveCommand(() -> -nthKeepSign(pilotController.getLeftY(), 2),
                        () -> -nthKeepSign(pilotController.getLeftX(), 2),
                        () -> -nthKeepSign(pilotController.getRightX(), 2),
                        SWERVE_MAX_LINEAR_VELOCITY,
                        SWERVE_MAX_ANGULAR_VELOCITY,
                        drivetrain, robotOrientedMod));
        slowMod.whileTrue(new TeleopDriveCommand(() -> -nthKeepSign(pilotController.getLeftY(), 2),
                () -> -nthKeepSign(pilotController.getLeftX(), 2),
                () -> -nthKeepSign(pilotController.getRightX(), 2),
                SWERVE_SLOW_LINEAR_VELOCITY,
                SWERVE_SLOW_ANGULAR_VELOCITY,
                drivetrain, robotOrientedMod));
        pilotController.rightBumper().whileTrue(climb());

        // Copilot controls
        coPilotController.a().onTrue(coralAlignFeeder().andThen(limitAccel(Level.FEEDER, slowMod.getAsBoolean())));
        coPilotController.b().onTrue(coralAlignL2().alongWith(limitAccel(Level.L2_CORAL, slowMod.getAsBoolean())));
        coPilotController.x().onTrue(coralAlignL3().alongWith(limitAccel(Level.L3_CORAL, slowMod.getAsBoolean())));
        coPilotController.y().onTrue(coralAlignL4().alongWith(limitAccel(Level.L4_CORAL, slowMod.getAsBoolean())));

        coPilotController.rightTrigger().whileTrue(coralIn());
        coPilotController.rightBumper().whileTrue(coralOut()); 

        System.out.print(DriverStation.getMatchTime() <= 20);

        // Trigger endgame = new Trigger(() -> DriverStation.getMatchTime() <= 20); 
        // endgame.onFalse(new InstantCommand(() -> leds.setProgressSupplier(() -> elevator.getHeight() / Level.L4_CORAL.height)));
        // endgame.onTrue(new InstantCommand(() -> leds.setPattern(LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(InchesPerSecond.of(20), Inches.of(1)))));
    }

    public void setTestBindings() {
        pilotController.a().onTrue(manualElevatorUp());

        
        coPilotController.povLeft().onTrue(new CoralIntakeAngleCommand(coralIntake,
                CoralIntake.TargetAngle.BARGE));
        coPilotController.povRight().onTrue(new CoralIntakeAngleCommand(coralIntake,
                CoralIntake.TargetAngle.L4_ANGLE));
        coPilotController.povUp().onTrue(new CoralIntakeAngleCommand(coralIntake,
                CoralIntake.TargetAngle.SOURCE_ANGLE));
    }

    @Override
    public void robotInit() {
        leds.setProgressSupplier(() -> elevator.getHeight() / Level.L4_CORAL.height);
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
        autoChooser.getSelected().schedule();
    }

    @Override
    public void teleopInit() {
        // FunctionalCommand goToZero = new FunctionalCommand(elevator::goHome, () -> {
        // }, (b) -> {
        // }, elevator::isHome, elevator);
        // CommandScheduler.getInstance().schedule(goToZero);
        new InstantCommand(elevator::stop).schedule();
        clearCommandBindings();
        setTeleopBindings();
    }

    @Override
    public void testInit() {
        // FunctionalCommand goToZero = new FunctionalCommand(elevator::goHome, () -> {
        // }, (b) -> {
        // }, elevator::isHome, elevator);
        // CommandScheduler.getInstance().schedule(goToZero);
        clearCommandBindings();
        setTestBindings();
    }

    private double nthKeepSign(double num, int n) {
        return Math.copySign(Math.pow(num, n), num);
    }

}
