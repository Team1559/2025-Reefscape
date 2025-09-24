package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberIo2025 extends ClimberIo {
    private static final double MOTOR_VOLTS = 6;

    private final SparkFlex motor;
    private final DigitalInput limitSwitch;

    public ClimberIo2025(String name, SparkFlex motor, DigitalInput limitSwitch) {
        super(name);
        this.motor = motor;
        this.limitSwitch = limitSwitch;

        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(false);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void run() {
        if (limitSwitch.get()) {
            super.run();
            motor.setVoltage(MOTOR_VOLTS);
        }
    }

    @Override
    public void stop() {
        super.stop();
        motor.stopMotor();
    }
    @Override
    protected void updateInputs(ClimberInputs inputs) {
        inputs.isDone = !limitSwitch.get();
        inputs.motorCurrent = motor.getOutputCurrent();
    }
}
