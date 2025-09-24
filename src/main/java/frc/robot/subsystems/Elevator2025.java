package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.subsystems.elevator.Elevator;

public class Elevator2025 extends Elevator {
    private static final int MOTOR_ID = 21;

    public Elevator2025() {
        super("Elevator", new ElevatorIo2025("IO", new SparkFlex(MOTOR_ID, MotorType.kBrushless)));
    }

    public enum Level {
        FEEDER(0),
        L1_CORAL(0),
        L2_CORAL(.165),
        L3_CORAL(.578),
        L4_CORAL(1.35),
        
        L2_ALGAE(.454),
        L3_ALGAE(.818);
        public final double height;

        Level(double heightOffset) {
            this.height = heightOffset;
        }
    }
}
