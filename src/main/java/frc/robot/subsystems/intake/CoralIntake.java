package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralIntake extends Intake {
    private static final int INTAKE_MOTOR_ID = 2; //TODO: change this to actual motor id
    private static final int ANGLE_MOTOR_ID = 2; //TODO: change this to actual motor id
    private static final int ANGLE_ENCODER_ID = 2; //TODO: change this to real encoder id
    private static final Rotation2d ANGLE_ENCODER_OFFSET = new Rotation2d(); //TODO: put real angle encoder offset

    public CoralIntake() {
        super("CoralIntake", createIo());
    }

    private static IntakeIo createIo() {
        SparkFlex intakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkFlex angleMotor = new SparkFlex(ANGLE_MOTOR_ID, MotorType.kBrushless);
        CANcoder angleEncoder = new CANcoder(ANGLE_ENCODER_ID);
        return new CoralIntakeIo("Io", intakeMotor, angleMotor, angleEncoder, ANGLE_ENCODER_OFFSET);
    }
}
