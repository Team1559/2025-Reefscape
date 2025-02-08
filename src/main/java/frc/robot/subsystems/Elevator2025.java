package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.subsystems.elevator.Elevator;

public class Elevator2025 extends Elevator {
    private static final int MOTOR_ID = 21;
    
    public Elevator2025() {
        super("Elevator", new ElevatorIo2025("IO", new SparkFlex(MOTOR_ID, MotorType.kBrushless)));
    }
}
