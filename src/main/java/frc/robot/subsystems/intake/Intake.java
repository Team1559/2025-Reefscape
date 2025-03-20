package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.LoggableSubsystem;
import frc.robot.subsystems.intake.IntakeIo.IntakeInputs;

public class Intake extends LoggableSubsystem {
    private final IntakeIo io;
    private Rotation2d targetAngle;
    public Intake(String name, IntakeIo io) {
        super(name);
        this.io = io;
        this.targetAngle = new Rotation2d(0);
        addIo(io);
    }

    public void run(boolean forward) {
        run(forward, false);
    }

    public void run(boolean forward, boolean slow){
        io.run(forward, slow);
    }

    public void setAngle(Rotation2d targetAngle) {
        io.setAngle(targetAngle);
        this.targetAngle = targetAngle;
    }

    public void stop() {
        io.stop();
    }

    public Rotation2d getCurrentAngle() {
        return io.getInputs().currentAngle;
    }

    public boolean isAtTargetAngle(Rotation2d tolerance) {
        IntakeInputs inputs = io.getInputs();
        return Math.abs(inputs.currentAngle.minus(targetAngle).getDegrees()) < tolerance.getDegrees();
    }
}
