// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

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

public class Robot extends TimedRobot {
    private final CommandXboxController pilotController;
    private final SwerveDrive swerveDrive;
    private final String canivoreBusName = "1559Canivore";
    // private final CommandXboxController coPilotController;

    public Robot() {
        pilotController = new CommandXboxController(0);
        swerveDrive = createSwerveDrive();
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
        SwerveModuleIo swerveFL = createSwerveModule(1, 0, 2, Rotation2d.fromRotations(-0.40087890625),
                new Translation2d(swerveModuleX, swerveModuleY));
        SwerveModuleIo swerveFR = createSwerveModule(4, 3, 5, Rotation2d.fromRotations(0.052001953125),
                new Translation2d(swerveModuleX, -swerveModuleY));
        SwerveModuleIo swerveRL = createSwerveModule(10, 9, 11, Rotation2d.fromRotations(-0.444091796875),
                new Translation2d(-swerveModuleX, swerveModuleY));
        SwerveModuleIo swerveRR = createSwerveModule(7, 6, 8, Rotation2d.fromRotations(-0.240234375),
                new Translation2d(-swerveModuleX, -swerveModuleY));


        // TODO: make gyro class, log stuff, etc
        Pigeon2 gyro = new Pigeon2(12, canivoreBusName);
        Supplier<Rotation2d> yaw = () -> Rotation2d.fromDegrees(gyro.getYaw(false).getValueAsDouble());
        return new SwerveDrive(yaw, swerveFR, swerveFL, swerveRR, swerveRL);
    }

    public SwerveModuleIo createSwerveModule(int steerMotorId, int driveMotorId, int canCoderId,
            Rotation2d canCoderOffset, Translation2d locationOffset) {

        TalonFX steerMotor = new TalonFX(steerMotorId, canivoreBusName);
        Slot0Configs steerMotorPid = new Slot0Configs().withKP(5);

        TalonFX driveMotor = new TalonFX(driveMotorId, canivoreBusName);

        Slot0Configs driveMotorPid = new Slot0Configs().withKV(12 / (6380.0 / 60)); // TODO: add the kd

        CANcoder canCoder = new CANcoder(canCoderId, canivoreBusName);

        driveMotor.setPosition(0);
        steerMotor.setPosition(canCoder.getAbsolutePosition().getValueAsDouble()*12.8 - canCoderOffset.getRotations()*12.8);
        System.out.println(canCoderId + ": " + canCoder.getAbsolutePosition().getValueAsDouble());
        return new SdsMk4Module(steerMotor, driveMotor, canCoder, SdsMk4Module.GearRatio.L2, canCoderOffset,
                locationOffset, steerMotorPid, driveMotorPid);
    }
}
