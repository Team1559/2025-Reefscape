package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber2025 extends Climber {
    private static final int MOTOR_ID = 89;
    
    public Climber2025() {
        super("Climber", new ClimberIo2025("IO", new SparkFlex(MOTOR_ID, MotorType.kBrushless)));
    }
}
