// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.subsystems.swerve.SdsMk4Module;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModuleIo;
import frc.robot.subsystems.swerve.SdsMk4Module.GearRatio;

public class Robot extends LoggedRobot {
    private final CommandXboxController pilotController;
    private final SwerveDrive swerveDrive;
    private final String canivoreBusName = "1559Canivore";
    // private final CommandXboxController coPilotController;

    public Robot() {
        pilotController = new CommandXboxController(0);
        swerveDrive = createSwerveDrive();
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("hi/test", ":)"); //Leave as easter egg
        // coPilotController = new CommandXboxController(1);
    }

    @Override
    public void robotInit() {
        swerveDrive.setDefaultCommand(new TeleopDriveCommand(pilotController::getLeftY, pilotController::getLeftX,
                pilotController::getRightX, 1, 1, swerveDrive));
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

    public SwerveDrive createSwerveDrive() {
        double swerveModuleX = Units.inchesToMeters(12);
        double swerveModuleY = Units.inchesToMeters(12);
        SwerveModuleIo frontLeft = createSwerveModule(1, 0, 2, Rotation2d.fromRadians(-1.88),
                new Translation2d(swerveModuleX, swerveModuleY));
        SwerveModuleIo frontRight = createSwerveModule(4, 3, 5, Rotation2d.fromRadians(-1.43),
                new Translation2d(swerveModuleX, -swerveModuleY));
        SwerveModuleIo rearLeft = createSwerveModule(10, 9, 11, Rotation2d.fromRadians(-1.5),
                new Translation2d(-swerveModuleX, swerveModuleY));
        SwerveModuleIo rearRight = createSwerveModule(7, 6, 8, Rotation2d.fromRadians(-0.01),
                new Translation2d(-swerveModuleX, -swerveModuleY));

        // TODO: make gyro class, log stuff, etc
        Pigeon2 gyro = new Pigeon2(12, canivoreBusName);
        Supplier<Rotation2d> yaw = () -> Rotation2d.fromDegrees(gyro.getYaw(false).getValueAsDouble());
        return new SwerveDrive(yaw, frontLeft, frontRight, rearLeft, rearRight);
    }

    public SwerveModuleIo createSwerveModule(int steerMotorId, int driveMotorId, int canCoderId,
            Rotation2d canCoderOffset, Translation2d locationOffset) {
                
        CANcoder canCoder = new CANcoder(canCoderId, canivoreBusName);
        TalonFX steerMotor = new TalonFX(steerMotorId, canivoreBusName);
        TalonFX driveMotor = new TalonFX(driveMotorId, canivoreBusName);
        
        Slot0Configs steerMotorPid = new Slot0Configs().withKP(60);
        Slot0Configs driveMotorPid = new Slot0Configs().withKV(12 / (6380.0 / 60)); // TODO: add the kd

        
        driveMotor.setPosition(0);
                
        return new SdsMk4Module(locationOffset, steerMotor, steerMotorPid, driveMotor, driveMotorPid, GearRatio.L2, canCoder, canCoderOffset);
    }
}
