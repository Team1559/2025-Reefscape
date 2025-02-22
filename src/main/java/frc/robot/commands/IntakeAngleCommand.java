package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeAngleCommand extends Command {
    private final Intake intake;
    private final Rotation2d targetAngle;
    private final Rotation2d tolerance;

    public IntakeAngleCommand(Intake intake, Rotation2d targetAngle, Rotation2d tolerance) {
        this.intake = intake;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return intake.isAtTargetAngle(tolerance);
    }
}
