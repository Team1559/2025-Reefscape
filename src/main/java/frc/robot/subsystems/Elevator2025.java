package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.subsystems.elevator.Elevator;

public class Elevator2025 extends Elevator {
    private static final int MOTOR_ID = 89; //FIXME set this to the real motor id
    
    public Elevator2025() {
        super(new ElevatorIo2025("/Elevator", new SparkFlex(MOTOR_ID, MotorType.kBrushless)));
    }
}
