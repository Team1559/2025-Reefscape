package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;

import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimberIo2025 extends ClimberIo {
    private static final double MOTOR_VOLTS = 2;

    private final SparkFlex motor;
    private final SparkLimitSwitch limitSwitch;

    public ClimberIo2025(String name, SparkFlex motor) {
        super(name);
        this.motor = motor;
        this.limitSwitch = motor.getForwardLimitSwitch();

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(false);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } 

    @Override
    public void run() {
        super.run();
        motor.setVoltage(MOTOR_VOLTS);
    }

    @Override
    public void testReverse(){
        super.testReverse();
        if(!DriverStation.isTestEnabled()){
                DriverStation.reportWarning("DO NOT REVERSE CLIMBER!!!", false);
        }else{
            motor.setVoltage(-MOTOR_VOLTS);
        }
    }

    @Override
    public void stop() {
        super.stop();
        motor.stopMotor();
    }
    @Override
    protected void updateInputs(ClimberInputs inputs) {
        inputs.isDone = limitSwitch.isPressed();
        inputs.motorCurrent = motor.getOutputCurrent();
    }
}
