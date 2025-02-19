package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.CoralIntake;
import frc.robot.subsystems.intake.CoralIntake.TargetAngle;

public class CoralIntakeAngleCommand extends IntakeAngleCommand {
    private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3);

    public CoralIntakeAngleCommand(CoralIntake intake, TargetAngle levelangle) {
        super(intake, levelangle.angle, TOLERANCE);
    }
}
