package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.swerve.SwerveDrive;
import frc.lib.subsystems.vision.LimelightCameraIo;
import frc.lib.subsystems.vision.Vision;
import frc.lib.subsystems.vision.VisionCameraIo;

public class Vision2025 extends Vision {
    private static final boolean VISION_ENABLED = true;

    public Vision2025(SwerveDrive swerveDrive) {
        super("Vision", swerveDrive, cameras(swerveDrive));
    }

    private static VisionCameraIo[] cameras(SwerveDrive swerveDrive) {
        if (VISION_ENABLED) {
            Supplier<Rotation2d> yaw = () -> swerveDrive.getPosition().getRotation();
            VisionCameraIo frontRight = new LimelightCameraIo("FrontRight", "limelight-frontr", yaw);
            VisionCameraIo frontLeft = new LimelightCameraIo("FrontLeft", "limelight-frontl", yaw);
            VisionCameraIo rear = new LimelightCameraIo("Rear", "limelight", yaw);
            VisionCameraIo frontCenter = new LimelightCameraIo("FrontCenter", "limelight-frontc", yaw); 
            return new VisionCameraIo[] { frontRight, rear, frontLeft, frontCenter };
        } else {
            return new VisionCameraIo[] {};
        }
    }
}
