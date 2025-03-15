package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Climber2025 extends Climber {
    private static final int MOTOR_ID = 27;
    private static final int LIMIT_SWITCH_PORT = 2;
    
    public Climber2025() {
        super("Climber", new ClimberIo2025("IO", new SparkFlex(MOTOR_ID, MotorType.kBrushless), new DigitalInput(LIMIT_SWITCH_PORT)));
    }
}
