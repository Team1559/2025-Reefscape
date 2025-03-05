package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.elevator.ElevatorIo;

public class ElevatorIo2025 extends ElevatorIo {
    private static final double GEAR_RATIO = 15;
    private static final int GEAR_TEETH = 13;
    private static final double RACK_SPACING = Units.inchesToMeters(3D / 8D);
    private static final double MAX_HEIGHT = Units.inchesToMeters(55);
    private static final double MAX_VELOCITY = 1;
    private static final double MAX_ACCEL = MAX_VELOCITY/.5;
    private static final double GRAVITY_FEEDFORWARD =.324; 
    private static final int NUM_STAGES = 2;
    private static final double ELEVATOR_DEADBAND = .001;

    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController motorController;
    private final SparkLimitSwitch limitSwitch;

    private boolean lastLimitSwitchState = false;
    private double targetHeight;

    public ElevatorIo2025(String name, SparkFlex motor) {
        super(name);
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.motorController = motor.getClosedLoopController();
        this.limitSwitch = motor.getReverseLimitSwitch();

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.closedLoop.maxMotion.maxVelocity(heightToMotorRotations(MAX_VELOCITY) * 60);
        motorConfig.closedLoop.maxMotion.maxAcceleration(heightToMotorRotations(MAX_ACCEL) * 60);

        motorConfig.closedLoop.pid(.1, 0, 0); // TODO: set these later
        motorConfig.inverted(false);

        motorConfig.smartCurrentLimit(200, 200);//TODO: DON'T SAVE THIS
        motorConfig.softLimit.forwardSoftLimit(heightToMotorRotations(MAX_HEIGHT));
        motorConfig.softLimit.forwardSoftLimitEnabled(true);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    protected void updateInputs(ElevatorInputs inputs) {
        inputs.isHome = limitSwitch.isPressed();
        inputs.currentPosition = motorRotationsToHeight(encoder.getPosition());
        inputs.motorCurrent = motor.getOutputCurrent();
        inputs.currentVelocity = motorRotationsToHeight(encoder.getVelocity())/60;
        inputs.temp = motor.getMotorTemperature();

        inputs.error = targetHeight - inputs.currentPosition;
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean isPressed = limitSwitch.isPressed();
        if (isPressed && !lastLimitSwitchState) { // Rising edge
            encoder.setPosition(0);
        }
        double error = targetHeight - getInputs().currentPosition;
        if (-ELEVATOR_DEADBAND > error && error < ELEVATOR_DEADBAND){
            setTargetPosition(targetHeight, ControlType.kPosition);
        }
        lastLimitSwitchState = isPressed;
    }

    @Override
    public void setTargetPosition(double targetHeight) {
        setTargetPosition(targetHeight, ControlType.kMAXMotionPositionControl);        
    }

    public void setTargetPosition(double targetHeight, ControlType controlType){
        super.setTargetPosition(targetHeight);
        this.targetHeight = targetHeight;
        Logger.recordOutput(getOutputLogPath("ControlType"), controlType);
        motorController.setReference(heightToMotorRotations(targetHeight), controlType,
                ClosedLoopSlot.kSlot0, GRAVITY_FEEDFORWARD);
    }

    private double motorRotationsToHeight(double motorRotations) {
        return motorRotations / GEAR_RATIO * GEAR_TEETH * RACK_SPACING * NUM_STAGES;
    }

    private double heightToMotorRotations(double height) {
        return height / RACK_SPACING / GEAR_TEETH * GEAR_RATIO / NUM_STAGES;
    }

    @Override
    public void goHome() {
        super.goHome();
        motor.setVoltage(-12 * 0.01);
    }
}