package frc.robot.subsystems;

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
    private static final double GEAR_RATIO = 25;
    private static final int GEAR_TEETH = 13;
    private static final double RACK_SPACING = Units.inchesToMeters(3D / 8D);
    private static final double MAX_HEIGHT = Units.inchesToMeters(55);
    private static final double MAX_VELOCITY = .5;//0.5;
    private static final double MAX_ACCEL = 0.5;
    private static final double GRAVITY_FEEDFORWARD = .0; // TODO: change this
    private static final int NUM_STAGES = 2;

    private final SparkFlex motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController motorController;
    private final SparkLimitSwitch limitSwitch;

    private boolean lastLimitSwitchState = false;



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
        motorConfig.closedLoop.pid(.08
        , 0, 0); // TODO: set these later
        motorConfig.inverted(false);

        motorConfig.softLimit.forwardSoftLimit(heightToMotorRotations(MAX_HEIGHT));
        motorConfig.softLimit.forwardSoftLimitEnabled(true);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    protected void updateInputs(ElevatorInputs inputs) {
        inputs.lowerLimitSwitch = limitSwitch.isPressed();
        inputs.currentPosition = motorRotationsToHeight(encoder.getPosition());
        inputs.motorCurrent = motor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean isPressed = limitSwitch.isPressed();
         if (isPressed && !lastLimitSwitchState) { //Rising edge
            encoder.setPosition(0);
        }

        lastLimitSwitchState = isPressed;
    }

    @Override
    public void setTargetPosition(double targetHeight) {
        super.setTargetPosition(targetHeight);
        motorController.setReference(heightToMotorRotations(targetHeight), ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0, GRAVITY_FEEDFORWARD);
    }

    private double motorRotationsToHeight(double motorRotations) {
        return motorRotations / GEAR_RATIO * GEAR_TEETH * RACK_SPACING * NUM_STAGES;
    }

    private double heightToMotorRotations(double height) {
        return height / RACK_SPACING / GEAR_TEETH * GEAR_RATIO / NUM_STAGES;
    }
}
