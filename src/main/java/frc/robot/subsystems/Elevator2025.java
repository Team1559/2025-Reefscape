package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.elevator.Elevator;

public class Elevator2025 extends Elevator {
    private static final int MOTOR_ID = 21;

    public Elevator2025() {
        super("Elevator", new ElevatorIo2025("IO", new SparkFlex(MOTOR_ID, MotorType.kBrushless)));
    }

    public enum Level {
        L1_CORAL(0),
        L2_CORAL(.153),
        L3_CORAL(.578),
        L4_CORAL(1.374),
        
        L2_ALGAE(1.36/2),
        L3_ALGAE(1.36*(3/4d));
        // TODO: Fix values

        public final double height;

        Level(double heightOffset) {
            this.height = heightOffset;
        }
    }
}
