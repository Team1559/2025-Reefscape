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
import edu.wpi.first.units.measure.Angle;

public class AlgaeIntakeIo extends IntakeIo {
    private static final double INTAKE_MOTOR_RPM = 89; // TODO: Put actual motor rpm
    private static final double INTAKE_MOTOR_ACCEL = 89; // TODO: Put actual motor accel
    private static final double GEAR_RATIO = 25;
    private static final double INTAKE_MASS = 89; // FIXME
    private static final double RADIUS_TO_COM = 89; // FIXME
    private static final double GRAVITY_ACCEL = 9.8; // FIXME
    private static final double MOTOR_STALL_TORQUE = 3.6;
    private static final double ANGLE_MOTOR_ACCEL = 89; // TODO: Put actual motor accel
    private static final double MAX_ANGLE_MOTOR_RPM = 89; // TODO: Put actual motor rpm

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
        leftIntakeMotorConfig.closedLoop.pid(0, 0, 0); // TODO: set these later
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
        angleMotorConfig.closedLoop.pid(0, 0, 0); // TODO: set these later
        angleMotorConfig.inverted(false);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.getEncoder().setPosition(getCurrentAngle().getRotations() * GEAR_RATIO);
    }

    @Override
    protected void updateInputs(IntakeInputs inputs) {
        inputs.currentAngle = getCurrentAngle();
    }

    @Override
    public void run(boolean forward) {
        super.run(forward);
        double velocity = forward ? INTAKE_MOTOR_RPM : -INTAKE_MOTOR_RPM;
        rightIntakeMotorController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
        leftIntakeMotorController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
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
        return angle.getCos() * GRAVITY_ACCEL * INTAKE_MASS * RADIUS_TO_COM / GEAR_RATIO / MOTOR_STALL_TORQUE;
    }

    @Override
    public void periodic() {
        super.periodic();
        if (targetAngle == null) {
            angleMotor.stopMotor();
        } else {
            angleMotorController.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                    gravityFeedForward(targetAngle));
        }
    }

    public double angleToMotorRotations(Rotation2d targetAngle) {
        return targetAngle.getRotations() * GEAR_RATIO;
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(absolutePosition.getValueAsDouble()).minus(angleEncoderOffset);
    }
}
