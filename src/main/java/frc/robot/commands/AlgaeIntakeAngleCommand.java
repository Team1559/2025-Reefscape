package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.intake.AlgaeIntake;
import frc.robot.subsystems.intake.AlgaeIntake.TargetAngle;

public class AlgaeIntakeAngleCommand extends IntakeAngleCommand {
    private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3);

    public AlgaeIntakeAngleCommand(AlgaeIntake intake, TargetAngle levelangle) {
        super(intake, levelangle.angle, TOLERANCE);
    }
}
