package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class CoralIntakeIo extends IntakeIo {
    private static final double INTAKE_MOTOR_RPM = 89; //TODO: Put actual motor rpm
    private static final double INTAKE_MOTOR_ACCEL = 89; //TODO: Put actual motor accel
    private static final double GEAR_RATIO = 25;
    private static final double INTAKE_MASS = 89; //FIXME
    private static final double RADIUS_TO_COM = 89; //FIXME
    private static final double GRAVITY_ACCEL = 89; //FIXME
    private static final double MOTOR_STALL_TORQUE = 3.6;
    private static final double ANGLE_MOTOR_ACCEL = 89; //TODO: Put actual motor accel
    private static final double MAX_ANGLE_MOTOR_RPM = 89; //TODO: Put actual motor rpm
    private final SparkFlex intakeMotor;
    private final SparkFlex angleMotor;
    private final CANcoder angleEncoder;
    private final StatusSignal<Angle> absolutePosition;
    private final Rotation2d angleEncoderOffset;
    private final SparkClosedLoopController intakeMotorController;
    private final SparkClosedLoopController angleMotorController;
    private Rotation2d targetAngle;

    public CoralIntakeIo(String name, SparkFlex intakeMotor, SparkFlex angleMotor, CANcoder angleEncoder, Rotation2d angleEncoderOffset) {
        super(name);
        this.intakeMotor = intakeMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.absolutePosition = angleEncoder.getAbsolutePosition();
        this.angleEncoderOffset = angleEncoderOffset;
        this.intakeMotorController = intakeMotor.getClosedLoopController();
        this.angleMotorController = angleMotor.getClosedLoopController();
        SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.closedLoop.maxMotion.maxAcceleration(INTAKE_MOTOR_ACCEL);
        intakeMotorConfig.closedLoop.pid(0, 0, 0); //TODO: set these later
        intakeMotorConfig.inverted(false);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.closedLoop.maxMotion.maxAcceleration(ANGLE_MOTOR_ACCEL);
        angleMotorConfig.closedLoop.maxMotion.maxVelocity(MAX_ANGLE_MOTOR_RPM);
        angleMotorConfig.closedLoop.pid(0, 0, 0); //TODO: set these later
        angleMotorConfig.inverted(false);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.getEncoder().setPosition(getCurrentAngle().getRotations()*GEAR_RATIO);
        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        angleEncoder.getConfigurator().apply(angleEncoderConfig);
    }

    @Override
    protected void updateInputs(IntakeInputs inputs) {
    inputs.currentAngle = getCurrentAngle();    
    }
    @Override
    public void run(boolean forward) {
        super.run(forward);
        intakeMotorController.setReference(INTAKE_MOTOR_RPM, ControlType.kMAXMotionVelocityControl);
    }
    @Override
    public void stop() {
        super.stop();
        intakeMotor.stopMotor();
    }
    @Override
    public void setAngle(Rotation2d targetAngle) {
        super.setAngle(targetAngle);
        this.targetAngle = targetAngle;
    }
    public double gravityFeedForward(Rotation2d angle) {
        return angle.getCos()*INTAKE_MASS*RADIUS_TO_COM/GEAR_RATIO/MOTOR_STALL_TORQUE;
    }
    @Override
    public void periodic() {
        super.periodic();
        if (targetAngle == null) {
            angleMotor.stopMotor();
        } else {
            angleMotorController.setReference(0, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, gravityFeedForward(targetAngle));
        }
    }
    public double angleToMotorRotations(Rotation2d targetAngle) {
        return targetAngle.getRotations()*GEAR_RATIO;
    }
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(absolutePosition.getValueAsDouble()).minus(angleEncoderOffset);
    }
}   
 