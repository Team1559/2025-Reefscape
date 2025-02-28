package frc.robot.subsystems.intake;

import java.util.prefs.BackingStoreException;

import org.opencv.ml.StatModel;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class CoralIntakeIo extends IntakeIo {
    private static final double INTAKE_MASS = Units.lbsToKilograms(7);
    private static final double RADIUS_TO_COM = Units.inchesToMeters(4.3);
    private static final double GRAVITY_ACCEL = 9.81;
    private static final double MOTOR_STALL_TORQUE = 3.6;
    private static final double BATTERY_VOLTAGE = 12;
    
    private static final double MOMENT_OF_INERTIA = RADIUS_TO_COM * INTAKE_MASS * .9;//.9 -> fudge

    private static final double INTAKE_MOTOR_GEAR_RATIO = 25;
    private static final double INTAKE_MOTOR_RPM = 120 * INTAKE_MOTOR_GEAR_RATIO; //2 rev/s mechanism
    private static final double INTAKE_MOTOR_ACCEL = INTAKE_MOTOR_RPM / .5;// 0.5s 

    private static final double ANGLE_GEAR_RATIO = 25;
    private static final double MAX_ANGLE_MOTOR_RPM = 12 * ANGLE_GEAR_RATIO;
    private static final double 
    ANGLE_MOTOR_ACCEL = MAX_ANGLE_MOTOR_RPM / .5;
    
    private final SparkFlex intakeMotor;
    private final SparkFlex angleMotor;
    private final CANcoder angleEncoder;
    
    private final SparkClosedLoopController intakeMotorController;
    private final SparkClosedLoopController angleMotorController;
    
    private final Rotation2d angleEncoderOffset;
    
    private final StatusSignal<Angle> absolutePosition;
    private Rotation2d targetAngle;

    public CoralIntakeIo(String name, SparkFlex intakeMotor, SparkFlex angleMotor, CANcoder angleEncoder,
            Rotation2d angleEncoderOffset) {
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
        intakeMotorConfig.closedLoop.pid(0, 0, 0); // TODO: set these later
        intakeMotorConfig.inverted(false);
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CANcoderConfiguration angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        angleEncoder.getConfigurator().apply(angleEncoderConfig);

        SparkFlexConfig angleMotorConfig = new SparkFlexConfig();
        angleMotorConfig.idleMode(IdleMode.kBrake);
        angleMotorConfig.closedLoop.maxMotion.maxAcceleration(ANGLE_MOTOR_ACCEL);
        angleMotorConfig.closedLoop.maxMotion.maxVelocity(MAX_ANGLE_MOTOR_RPM);
        angleMotorConfig.closedLoop.pid(.16, 0, 0); // TODO: set these later
        angleMotorConfig.inverted(true);
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.getEncoder().setPosition(getCurrentAngle().getRotations() * ANGLE_GEAR_RATIO);
    }

    @Override
    protected void updateInputs(IntakeInputs inputs) {
        inputs.currentAngle = getCurrentAngle();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.currentAngleFromMotor = motorRotationsToAngle(angleMotor.getEncoder().getPosition());
    }

    @Override
    public void run(boolean forward) {
        super.run(forward);
        double velocity = forward ? INTAKE_MOTOR_RPM : -INTAKE_MOTOR_RPM;
        intakeMotorController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
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
        return angle.getCos() * GRAVITY_ACCEL * MOMENT_OF_INERTIA/ ANGLE_GEAR_RATIO / MOTOR_STALL_TORQUE * BATTERY_VOLTAGE;// Upper limit : 1
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

    public Rotation2d motorRotationsToAngle(double rotations){
        return Rotation2d.fromRotations(rotations/ANGLE_GEAR_RATIO);
    }

    private Rotation2d getCurrentAngle() {
        absolutePosition.refresh();
        return Rotation2d.fromRotations(absolutePosition.getValueAsDouble()).minus(angleEncoderOffset);
    }
}
