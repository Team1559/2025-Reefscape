package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SdsMk4Module implements SwerveModuleIo {

    public enum GearRatio {
        L1(50d / 14 * 19 / 25 * 45 / 15),
        L2(50d / 14 * 17 / 27 * 45 / 15),
        L3(50d / 14 * 16 / 28 * 45 / 15),
        L4(48d / 16 * 16 / 28 * 45 / 15);

        private final double value;

        private GearRatio(double value) {
            this.value = value;
        }
    }

    // TODO: make WHEEL_RADIUS a parameter
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final GearRatio driveGearRatio;
    private final TalonFX steerMotor;
    private final TalonFX driveMotor;
    private final CANcoder cancoder;
    private final Rotation2d cancoderOffset;
    private final Translation2d location;

    public SdsMk4Module(Translation2d location, TalonFX steerMotor, Slot0Configs steerMotorPid, TalonFX driveMotor,
            Slot0Configs driveMotorPid, GearRatio driveGearRatio, CANcoder cancoder,
            Rotation2d cancoderOffset) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.cancoder = cancoder;
        this.driveGearRatio = driveGearRatio;
        this.cancoderOffset = cancoderOffset;
        this.location = location;

        steerMotor.getConfigurator().apply(new TalonFXConfiguration());
        steerMotor.getConfigurator().apply(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
        steerMotor.getConfigurator().apply(steerMotorPid);
        steerMotor.getConfigurator().apply(new FeedbackConfigs().withRemoteCANcoder(cancoder));
        ClosedLoopGeneralConfigs clgConfig = new ClosedLoopGeneralConfigs();
        clgConfig.ContinuousWrap = true;
        steerMotor.getConfigurator().apply(clgConfig);

        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        driveMotor.getConfigurator().apply(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
        driveMotor.getConfigurator().apply(driveMotorPid);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    @Override
    public void setSpeed(double speed) {
        VelocityVoltage control = new VelocityVoltage(
                speed * (1 / (2 * WHEEL_RADIUS * Math.PI)) * driveGearRatio.value);
        driveMotor.setControl(control);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        PositionVoltage control = new PositionVoltage(angle.plus(cancoderOffset).getRotations());
        steerMotor.setControl(control);
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public double getSpeed() {
        return driveMotor.getVelocity().getValueAsDouble() / driveGearRatio.value * (2 * WHEEL_RADIUS * Math.PI);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble()).minus(cancoderOffset);
    }

    @Override
    public double getDistanceTraveled() {
        return driveMotor.getPosition().getValueAsDouble() / driveGearRatio.value * (2 * WHEEL_RADIUS * Math.PI);
    }

    @Override
    public double getSteerMotorTemperature() {
        return steerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public double getDriveMotorTemperature() {
        return driveMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public double getSteerMotorCurrent() {
        return steerMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getDriveMotorCurrent() {
        return driveMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getSteerMotorVelocity() {
        return steerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getDriveMotorVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }
}
