package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralIntake extends Intake {
    private static final int INTAKE_MOTOR_ID = 26; 
    private static final int ANGLE_MOTOR_ID = 22;
    private static final int ANGLE_ENCODER_ID = 27; 
    private static final Rotation2d ANGLE_ENCODER_OFFSET = Rotation2d.fromRadians(.904);

    public CoralIntake() {
        super("CoralIntake", createIo());
    }

    public enum TargetAngle {
        L1_ANGLE(Rotation2d.fromRadians(-.574)),
        L2_ANGLE(Rotation2d.fromRadians(-.593)),
        L3_ANGLE(Rotation2d.fromRadians(-.593)),
        L4_ANGLE(Rotation2d.fromRadians(-.94)),
        
        SOURCE_ANGLE(Rotation2d.fromRadians(0.570));
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
