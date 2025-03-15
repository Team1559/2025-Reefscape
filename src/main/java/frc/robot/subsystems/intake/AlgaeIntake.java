package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeIntake extends Intake {
    private static final int LEFT_INTAKE_MOTOR_ID = 23; 
    private static final int RIGHT_INTAKE_MOTOR_ID = 25;
    private static final int ANGLE_MOTOR_ID = 24; 
    private static final int ANGLE_ENCODER_ID = 28;
    private static final Rotation2d ANGLE_ENCODER_OFFSET = Rotation2d.fromRadians(1.017); 
    
    public AlgaeIntake() {
        super("AlgaeIntake", createIo());
    }
    public enum TargetAngle {
        STOWED(Rotation2d.fromRadians(1.434)),
        REEF(Rotation2d.fromDegrees(-6)),
        PROCESSOR(Rotation2d.fromDegrees(-6)),
        FLOOR(Rotation2d.fromRadians(-.58)),
        BARGE(Rotation2d.fromRadians(.691));

        public final Rotation2d angle;
        TargetAngle(Rotation2d angle) {
            this.angle = angle;
        }
    }

    private static IntakeIo createIo() {
        SparkMax rightIntakeMotor = new SparkMax(RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMax leftIntakeMotor = new SparkMax(LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMax angleMotor = new SparkMax(ANGLE_MOTOR_ID, MotorType.kBrushless);
        CANcoder angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        return new AlgaeIntakeIo("Io", rightIntakeMotor, leftIntakeMotor, angleMotor, angleEncoder, ANGLE_ENCODER_OFFSET);
    }
}
