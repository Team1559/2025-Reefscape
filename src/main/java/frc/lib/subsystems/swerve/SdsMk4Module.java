package frc.lib.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class SdsMk4Module implements SwerveModuleIo {

    public enum ModuleType {
        MK4_L1(50d / 14 * 19 / 25 * 45 / 15, InvertedValue.CounterClockwise_Positive),
        MK4_L2(50d / 14 * 17 / 27 * 45 / 15, InvertedValue.CounterClockwise_Positive),
        MK4_L3(50d / 14 * 16 / 28 * 45 / 15, InvertedValue.CounterClockwise_Positive),
        MK4_L4(48d / 16 * 16 / 28 * 45 / 15, InvertedValue.CounterClockwise_Positive),

        MK4i_L1(-50d / 14 * 19 / 25 * 45 / 15, InvertedValue.Clockwise_Positive),
        MK4i_L2(-50d / 14 * 17 / 27 * 45 / 15, InvertedValue.Clockwise_Positive),
        MK4i_L3(-50d / 14 * 16 / 28 * 45 / 15, InvertedValue.Clockwise_Positive);

        private final double driveRatio;
        private final InvertedValue steerDirection;

        private ModuleType(double driveRatio, InvertedValue steerDirection) {
            this.driveRatio = driveRatio;
            this.steerDirection = steerDirection;
        }
    }

    // TODO: make WHEEL_RADIUS a parameter
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final ModuleType driveGearRatio;
    private final TalonFX steerMotor;
    private final TalonFX driveMotor;
    private final CANcoder cancoder;
    private final Rotation2d cancoderOffset;
    private final Translation2d location;
    private final String name;
    private double setSpeed;
    private Rotation2d setAngle;
    private final StatusSignal<AngularVelocity> driveMotorVelocity;
    private final StatusSignal<Angle> canCoderAbsolutePosition;
    private final StatusSignal<Angle> driveMotorPosition;
    private final StatusSignal<Temperature> steerMotorTemperature;
    private final StatusSignal<Temperature> driveMotorTemperature;
    private final StatusSignal<Current> steerMotorCurrent;
    private final StatusSignal<Current> driveMotorCurrent;

    public SdsMk4Module(String name, Translation2d location, ModuleType moduleType, TalonFX steerMotor,
            Slot0Configs steerMotorPid,
            TalonFX driveMotor,
            Slot0Configs driveMotorPid, CANcoder cancoder,
            Rotation2d cancoderOffset) {
        this.name = name;
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.cancoder = cancoder;
        this.driveGearRatio = moduleType;
        this.cancoderOffset = cancoderOffset;
        this.location = location;

        steerMotor.getConfigurator().apply(new TalonFXConfiguration());
        steerMotor.getConfigurator().apply(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(moduleType.steerDirection));
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
        driveMotor.setPosition(0);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        driveMotorVelocity = driveMotor.getVelocity();
        canCoderAbsolutePosition = cancoder.getAbsolutePosition();
        driveMotorPosition = driveMotor.getPosition();
        driveMotorTemperature = driveMotor.getDeviceTemp();
        steerMotorTemperature = steerMotor.getDeviceTemp();
        driveMotorCurrent = driveMotor.getStatorCurrent();
        steerMotorCurrent = steerMotor.getStatorCurrent();

        driveMotorVelocity.setUpdateFrequency(50);
        canCoderAbsolutePosition.setUpdateFrequency(100);
        driveMotorPosition.setUpdateFrequency(100);
        driveMotorTemperature.setUpdateFrequency(5);
        steerMotorTemperature.setUpdateFrequency(5);
        driveMotorCurrent.setUpdateFrequency(10);
        steerMotorCurrent.setUpdateFrequency(10);
    }

    @Override
    public void setSpeed(double speed) {
        this.setSpeed = speed;
        VelocityVoltage control = new VelocityVoltage(
                speed * (1 / (2 * WHEEL_RADIUS * Math.PI)) * driveGearRatio.driveRatio);
        driveMotor.setControl(control);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.setAngle = angle;
        PositionVoltage control = new PositionVoltage(angle.plus(cancoderOffset).getRotations());
        steerMotor.setControl(control);
    }

    @Override
    public Translation2d getLocation() {
        return location;
    }

    @Override
    public double getSpeed() {
        return driveMotorVelocity.getValueAsDouble() / driveGearRatio.driveRatio * (2 * WHEEL_RADIUS * Math.PI);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(canCoderAbsolutePosition.getValueAsDouble()).minus(cancoderOffset);
    }

    @Override
    public double getDistanceTraveled() {
        return driveMotorPosition.getValueAsDouble() / driveGearRatio.driveRatio * (2 * WHEEL_RADIUS * Math.PI);
    }

    @Override
    public double getSteerMotorTemperature() {
        return steerMotorTemperature.getValueAsDouble();
    }

    @Override
    public double getDriveMotorTemperature() {
        return driveMotorTemperature.getValueAsDouble();
    }

    @Override
    public double getSteerMotorCurrent() {
        return steerMotorCurrent.getValueAsDouble();
    }

    @Override
    public double getDriveMotorCurrent() {
        return driveMotorCurrent.getValueAsDouble();
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public SwerveModuleState getSetpoint() {
        return new SwerveModuleState(setSpeed, setAngle);
    }

}
