package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class CoralIntake extends Intake {
    private static final int INTAKE_MOTOR_ID = 26; 
    private static final int ANGLE_MOTOR_ID = 22;
    private static final int ANGLE_ENCODER_ID = 27; 
    private static final Rotation2d ANGLE_ENCODER_OFFSET = new Rotation2d();

    public CoralIntake() {
        super("CoralIntake", createIo());
    }

    public enum TargetAngle {
        L1_ANGLE(Rotation2d.fromDegrees(30)),
        L2_ANGLE(Rotation2d.fromDegrees(20)),
        L3_ANGLE(Rotation2d.fromDegrees(50)),
        L4_ANGLE(Rotation2d.fromDegrees(60)); // FIXME: put real angles in

        public final Rotation2d angle;

        TargetAngle(Rotation2d angle) {
            this.angle = angle;
        }
    }

    private static IntakeIo createIo() {
        SparkFlex intakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkFlex angleMotor = new SparkFlex(ANGLE_MOTOR_ID, MotorType.kBrushless);
        CANcoder angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        return new CoralIntakeIo("Io", intakeMotor, angleMotor, angleEncoder, ANGLE_ENCODER_OFFSET);
    }
}
