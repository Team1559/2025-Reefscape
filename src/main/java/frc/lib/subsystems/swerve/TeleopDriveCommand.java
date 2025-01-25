package frc.lib.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDriveCommand extends Command {
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rSupplier;

    private final double maxLinearVelocity;
    private final double maxRotationalVelocity;

    private final SwerveDrive swerveDrive;

    private static final double DEADBAND = .10;

    public TeleopDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier,
            double maxLinearVelocity, double maxRotationalVelocity, SwerveDrive swerveDrive) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rSupplier = rSupplier;

        this.maxLinearVelocity = maxLinearVelocity;
        this.maxRotationalVelocity = maxRotationalVelocity;

        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double x = xSupplier.getAsDouble();
        double y = ySupplier.getAsDouble();
        double rotation = rSupplier.getAsDouble();

        double magnitude = Math.hypot(x, y);

        if (magnitude > 1) {
            x /= magnitude;
            y /= magnitude;
        } else if (magnitude < DEADBAND) {
            x = 0;
            y = 0;
        } else {
            double deadbandMagnitude = MathUtil.applyDeadband(magnitude, DEADBAND);
            x *= deadbandMagnitude/magnitude;
            y *= deadbandMagnitude/magnitude;
        }

        rotation = MathUtil.applyDeadband(rotation, DEADBAND);

        x *= maxLinearVelocity;
        y *= maxLinearVelocity;
        rotation *= maxRotationalVelocity;

        swerveDrive.driveFieldOriented(new ChassisSpeeds(x, y, rotation));
    }
}