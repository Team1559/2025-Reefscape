package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

public class AlgaeIntakeIo extends IntakeIo {
    
    private final SparkMax rightIntakeMotor;
    private final SparkMax leftIntakeMotor;
    private final CANcoder cancoder;

    public AlgaeIntakeIo(String name, SparkMax rightIntakeMotor, SparkMax leftIntakeMotor, CANcoder cancoder) {
        super(name);
        this.leftIntakeMotor = leftIntakeMotor;
        this.rightIntakeMotor = rightIntakeMotor;
        this.cancoder = cancoder;
    }
}
