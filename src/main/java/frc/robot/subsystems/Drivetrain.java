package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.subsystems.swerve.SdsMk4Module;
import frc.lib.subsystems.swerve.SwerveDrive;
import frc.lib.subsystems.swerve.SwerveModuleIo;
import frc.lib.subsystems.swerve.SdsMk4Module.ModuleType;

public class Drivetrain extends SwerveDrive {
    private static final String canivoreBusName = "1559_Canivore";

    public Drivetrain() {
        super(rotationSupplierSupplier(), moduleSupplier());
        setName("swerveDrive");
    }

    private static SwerveModuleIo createSwerveModule(String name, int steerMotorId, int driveMotorId,
            int canCoderId,
            Rotation2d canCoderOffset, Translation2d locationOffset) {

        CANcoder canCoder = new CANcoder(canCoderId, canivoreBusName);
        TalonFX steerMotor = new TalonFX(steerMotorId, canivoreBusName);
        TalonFX driveMotor = new TalonFX(driveMotorId, canivoreBusName);

        Slot0Configs steerMotorPid = new Slot0Configs().withKP(60);
        Slot0Configs driveMotorPid = new Slot0Configs().withKV(12 / (6380.0 / 60)); // TODO: add the kd

        return new SdsMk4Module(name, locationOffset, ModuleType.MK4i_L3, steerMotor, steerMotorPid, driveMotor,
                driveMotorPid,
                canCoder, canCoderOffset);
    }

    private static Supplier<Rotation2d> rotationSupplierSupplier() {
        Pigeon2 gyro = new Pigeon2(13, canivoreBusName);
        return () -> Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    private static SwerveModuleIo[] moduleSupplier() {
        double swerveModuleX = Units.inchesToMeters(12);
        double swerveModuleY = Units.inchesToMeters(12);
        SwerveModuleIo frontLeft = createSwerveModule("frontLeft", 1, 3, 2, Rotation2d.fromRadians(1.249),
                new Translation2d(swerveModuleX, swerveModuleY));
        SwerveModuleIo frontRight = createSwerveModule("frontRight", 4, 6, 5, Rotation2d.fromRadians(-1.231),
                new Translation2d(swerveModuleX, -swerveModuleY));
        SwerveModuleIo rearLeft = createSwerveModule("rearLeft", 10, 12, 11, Rotation2d.fromRadians(-2.251),
                new Translation2d(-swerveModuleX, swerveModuleY));
        SwerveModuleIo rearRight = createSwerveModule("rearRight", 7, 9, 8, Rotation2d.fromRadians(-1.574),
                new Translation2d(-swerveModuleX, -swerveModuleY));
        return new SwerveModuleIo[] { frontLeft, frontRight, rearLeft, rearRight };
    }

}
