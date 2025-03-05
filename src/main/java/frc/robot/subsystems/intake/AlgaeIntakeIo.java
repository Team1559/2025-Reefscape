package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class AlgaeIntakeIo extends IntakeIo {
    
    private static final double INTAKE_MASS = 4.536;
    private static final double RADIUS_TO_COM = Units.inchesToMeters(3.831);
    private static final double GRAVITY_ACCEL = 9.81;
    private static final double BATTERY_VOLTAGE = 12;

    private static final double MOTOR_STALL_TORQUE = 3.6;
    
    private static final double ANGLE_GEAR_RATIO = 75;
    private static final double MAX_ANGLE_MOTOR_RPM = 15 * ANGLE_GEAR_RATIO; 
    private static final double ANGLE_MOTOR_ACCEL = MAX_ANGLE_MOTOR_RPM/.5;
    
    private static final double INTAKE_GEAR_RATIO = 50;
    private static final double INTAKE_MOTOR_RPM = 120 * INTAKE_GEAR_RATIO;
    private static final double INTAKE_MOTOR_ACCEL = INTAKE_MOTOR_RPM/.1;
    private static final double INTAKE_MOTOR_MAX_RPM = 11000;
    private static final double INTAKE_MOTOR_VOLTAGE = INTAKE_MOTOR_RPM / INTAKE_MOTOR_MAX_RPM * BATTERY_VOLTAGE;
    private final SparkMax rightIntakeMotor;
    private final SparkMax leftIntakeMotor;
    private final SparkMax angleMotor;
    private final CANcoder angleEncoder;
    private final SparkClosedLoopController rightIntakeMotorController;
    private final SparkClosedLoopController leftIntakeMotorController;
    private final SparkClosedLoopController angleMotorController;
    private final StatusSignal<Angle> absolutePosition;
    private final Rotation2d angleEncoderOffset;

    private Rotation2d targetAngle;

    public AlgaeIntakeIo(String name, SparkMax rightIntakeMotor, SparkMax leftIntakeMotor, SparkMax angleMotor,
            CANcoder angleEncoder, Rotation2d angleEncoderOffset) {
        super(name);
        this.leftIntakeMotor = leftIntakeMotor;
        this.rightIntakeMotor = rightIntakeMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.absolutePosition = angleEncoder.getAbsolutePosition();
        this.rightIntakeMotorController = rightIntakeMotor.getClosedLoopController();
        this.leftIntakeMotorController = leftIntakeMotor.getClosedLoopController();
        this.angleMotorController = angleMotor.getClosedLoopController();
        this.angleEncoderOffset = angleEncoderOffset;

        SparkMaxConfig leftIntakeMotorConfig = new SparkMaxConfig();
        leftIntakeMotorConfig.idleMode(IdleMode.kBrake);
        leftIntakeMotorConfig.closedLoop.maxMotion.maxAcceleration(INTAKE_MOTOR_ACCEL);
        leftIntakeMotorConfig.inverted(false);
        leftIntakeMotor.configure(leftIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig().apply(leftIntakeMotorConfig);
        rightIntakeMotorConfig.inverted(true);
        rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        angleEncoder.getConfigurator().apply(angleEncoderConfig);

        SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.closedLoop.maxMotion.maxAcceleration(ANGLE_MOTOR_ACCEL);
        angleMotorConfig.closedLoop.maxMotion.maxVelocity(MAX_ANGLE_MOTOR_RPM);
        angleMotorConfig.closedLoop.pid(1, 0, 0); // TODO: set these later
        angleMotorConfig.inverted(true);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.getEncoder().setPosition(getCurrentAngle().getRotations() * ANGLE_GEAR_RATIO);
    }

    @Override
    protected void updateInputs(IntakeInputs inputs) {
        inputs.currentAngle = getCurrentAngle();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
    }

    @Override
    public void run(boolean forward) {
        super.run(forward);
        double voltage = forward ? INTAKE_MOTOR_VOLTAGE:-INTAKE_MOTOR_VOLTAGE;
        rightIntakeMotorController.setReference(voltage, ControlType.kVoltage);
        leftIntakeMotorController.setReference(voltage, ControlType.kVoltage);
    }

    @Override
    public void stop() {
        super.stop();
        rightIntakeMotor.stopMotor();
        leftIntakeMotor.stopMotor();
    }

    @Override
    public void setAngle(Rotation2d targetAngle) {
        super.setAngle(targetAngle);
        this.targetAngle = targetAngle;
    }

    public double gravityFeedForward(Rotation2d angle) {
        return angle.getCos() * GRAVITY_ACCEL * INTAKE_MASS * RADIUS_TO_COM / ANGLE_GEAR_RATIO / MOTOR_STALL_TORQUE * BATTERY_VOLTAGE;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (targetAngle == null) {
            angleMotor.stopMotor();
        } else {
            angleMotorController.setReference(angleToMotorRotations(targetAngle), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                    gravityFeedForward(getInputs().currentAngle));
        }
    }

    public double angleToMotorRotations(Rotation2d targetAngle) {
        return targetAngle.getRotations() * ANGLE_GEAR_RATIO;
    }

    public Rotation2d getCurrentAngle() {
        absolutePosition.refresh();
        return Rotation2d.fromRotations(absolutePosition.getValueAsDouble()).minus(angleEncoderOffset);
    }
}
