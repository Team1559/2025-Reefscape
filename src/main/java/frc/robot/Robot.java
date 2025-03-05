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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.subsystems.swerve.TeleopDriveCommand;
import frc.robot.commands.AlgaeIntakeAngleCommand;
import frc.robot.commands.ElevatorHeightCommand2025;
import frc.robot.subsystems.Elevator2025;
import frc.robot.subsystems.Elevator2025.Level;
import frc.robot.subsystems.SwerveDrive2025;
import frc.robot.subsystems.Vision2025;
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
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void clearControllers(){
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
    }

    public void setTeleopBindings(){
        drivetrain.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, SWERVE_MAX_LINEAR_VELOCITY, SWERVE_MAX_ANGULAR_VELOCITY, drivetrain));
    }

    public void setTestBindings(){
        coPilotController.a().onTrue(NamedCommands.getCommand("coralLevelOne"));
        coPilotController.b().and(coPilotController.rightTrigger().negate()).onTrue(NamedCommands.getCommand("coralLevelTwo"));
        coPilotController.b().and(coPilotController.rightTrigger()).onTrue(NamedCommands.getCommand("algaeLevelTwo"));
        coPilotController.x().and(coPilotController.rightTrigger().negate()).onTrue(NamedCommands.getCommand(("coralLevelThree")));
        coPilotController.x().and(coPilotController.rightTrigger()).onTrue(NamedCommands.getCommand("coralLevelFour"));

        coPilotController.y().onTrue(NamedCommands.getCommand("coralLevelFour"));
        // coPilotController.povDown().onTrue(manualElevatorUp);//.onTrue(elevatorHome);

        drivetrain.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, 5.21, 1.925, drivetrain));
        pilotController.a().onTrue(new InstantCommand(elevator::stop));//TODO: temp

        coPilotController.rightTrigger().whileTrue(new StartEndCommand(() -> algaeIntake.run(false), () -> algaeIntake.stop(), algaeIntake));
        coPilotController.rightBumper().whileTrue(new StartEndCommand(() -> algaeIntake.run(true), () -> algaeIntake.stop(), algaeIntake));
        
        coPilotController.leftTrigger().whileTrue(new StartEndCommand(() -> coralIntake.run(false), () -> coralIntake.stop(), coralIntake));
        coPilotController.leftBumper().whileTrue(new StartEndCommand(() -> coralIntake.run(true), () -> coralIntake.stop(), coralIntake));

        coPilotController.povUp().onTrue(new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.STOWED));
        coPilotController.povDown().onTrue(new AlgaeIntakeAngleCommand(algaeIntake, AlgaeIntake.TargetAngle.REEF));
        
        // coPilotController.povLeft().onTrue(new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L2_ANGLE));
        // coPilotController.povRight().onTrue(new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.L4_ANGLE));
        // coPilotController.povUp().onTrue(new CoralIntakeAngleCommand(coralIntake, CoralIntake.TargetAngle.SOURCE_ANGLE));
    }

    @Override
    public void robotInit() {
        Command coralLevelOne = new ElevatorHeightCommand2025(elevator, Level.L1_CORAL);
        Command coralLevelTwo = new ElevatorHeightCommand2025(elevator, Level.L2_CORAL);
        Command coralLevelThree = new ElevatorHeightCommand2025(elevator, Level.L3_CORAL);
        Command coralLevelFour = new ElevatorHeightCommand2025(elevator, Level.L4_CORAL);

        Command algaeLevelTwo = new ElevatorHeightCommand2025(elevator, Level.L2_ALGAE);
        Command algaeLevelThree = new ElevatorHeightCommand2025(elevator, Level.L3_ALGAE);

        Command elevatorHome = new InstantCommand(
                () -> elevator.goHome(),
                elevator);

        Command manualElevatorUp = new InstantCommand(
            () -> elevator.changeTargetPosition(.005), elevator
        );
        NamedCommands.registerCommand("coralLevelOne", coralLevelOne);
        NamedCommands.registerCommand("coralLevelTwo", coralLevelTwo);
        NamedCommands.registerCommand("coralLevelThree", coralLevelThree);
        NamedCommands.registerCommand("coralLevelFour", coralLevelFour);
        NamedCommands.registerCommand("algaeLevelTwo", algaeLevelTwo);
        NamedCommands.registerCommand("algaeLevelThree", algaeLevelThree);
        NamedCommands.registerCommand("elevatorHome", elevatorHome);
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

        clearControllers();
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
        clearControllers();
        setTestBindings();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
