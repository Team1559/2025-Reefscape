package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.LoggableSubsystem;

public class Intake extends LoggableSubsystem {
    private final IntakeIo io;

    public Intake(String name, IntakeIo io) {
        super(name);
        this.io = io;

        addIo(io);
    }

    public void run(boolean forward) {
        io.run(forward);
    }

    public void setAngle(Rotation2d targetAngle) {
        io.setAngle(targetAngle);
    }

    public void stop() {
        io.stop();
    }
}
