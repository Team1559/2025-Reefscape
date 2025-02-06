package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.swerve.SwerveDrive;
import frc.lib.subsystems.vision.LimelightCameraIo;
import frc.lib.subsystems.vision.Vision;
import frc.lib.subsystems.vision.VisionCameraIo;

public class Vision2025 extends Vision {

    public Vision2025(SwerveDrive swerveDrive) {
        super("Vision", swerveDrive, cameras(swerveDrive));
    }

    private static VisionCameraIo[] cameras(SwerveDrive swerveDrive) {
        Supplier<Rotation2d> yaw = () -> swerveDrive.getPosition().getRotation();
        VisionCameraIo frontRight = new LimelightCameraIo("FrontRight", "limelight-frontr", yaw);
        VisionCameraIo frontLeft = new LimelightCameraIo("FrontLeft", "limelight-frontl", yaw);
        // TODO: Add two more cameras! <3
        return new VisionCameraIo[] { frontRight, frontLeft };
        // eventual order: return new VisionCameraIo[] {frontRight, back, frontLeft,
        // front};
    }
}