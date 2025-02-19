package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;

public class AlgaeIntake extends Intake {
    private static final int LEFT_INTAKE_MOTOR_ID = 2; //TODO: change this to actual motor id
    private static final int RIGHT_INTAKE_MOTOR_ID = 2; //TODO: change this to actual motor id
    private static final int ANGLE_MOTOR_ID = 2; //TODO: change this to actual motor id
    private static final int ANGLE_ENCODER_ID = 2; //TODO: change this to real encoder id
    private static final Rotation2d ANGLE_ENCODER_OFFSET = new Rotation2d(); //TODO: put real angle encoder offset
    private static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);

    public AlgaeIntake() {
        super("AlgaeIntake", createIo());
    }

    private static IntakeIo createIo() {
        SparkMax rightIntakeMotor = new SparkMax(RIGHT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMax leftIntakeMotor = new SparkMax(LEFT_INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMax angleMotor = new SparkMax(ANGLE_MOTOR_ID, MotorType.kBrushless);
        CANcoder angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        return new AlgaeIntakeIo("Io", rightIntakeMotor, leftIntakeMotor, angleMotor, angleEncoder, ANGLE_ENCODER_OFFSET);
    }
}
