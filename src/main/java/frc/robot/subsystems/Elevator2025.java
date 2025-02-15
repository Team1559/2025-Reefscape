package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.lib.subsystems.elevator.Elevator;

public class Elevator2025 extends Elevator {
    private static final int MOTOR_ID = 21;
    private static final double HEIGHT_OFFSET = Units.inchesToMeters(35);

    public Elevator2025() {
        super("Elevator", new ElevatorIo2025("IO", new SparkFlex(MOTOR_ID, MotorType.kBrushless)), HEIGHT_OFFSET);
    }

    public enum IntakeOffset {
        ALGAE(Units.inchesToMeters(18)), 
        CORAL(Units.inchesToMeters(18));

        public final double heightOffset;

        IntakeOffset(double heightOffset) {
            this.heightOffset = heightOffset;
        }
    }

    public enum Level {
        L1(Units.inchesToMeters(18)),
        L2(Units.inchesToMeters(31.875)),
        L3(Units.inchesToMeters(41.625)),
        L4(Units.inchesToMeters(72));

        public final double heightOffset;

        Level(double heightOffset) {
            this.heightOffset = heightOffset;
        }
    }
}
