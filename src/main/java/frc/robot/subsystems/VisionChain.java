package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.swerve.SwerveDrive;
import frc.lib.subsystems.vision.LimelightCamera;
import frc.lib.subsystems.vision.Vision;
import frc.lib.subsystems.vision.VisionCameraIo;

public class VisionChain extends Vision{
    
    public VisionChain(SwerveDrive swerveDrive){
        super(swerveDrive, cameras(swerveDrive));
    }

    private static VisionCameraIo[] cameras(SwerveDrive swerveDrive){
        Supplier<Rotation2d> yaw = () -> swerveDrive.getPosition().getRotation();
        VisionCameraIo frontRight = new LimelightCamera("vision/frontRightCamera", "frontRight", yaw);
        VisionCameraIo frontLeft = new LimelightCamera("vision/frontLeftCamera", "frontLeft", yaw);
        //TODO: Add two more cameras! <3
        return new VisionCameraIo[] {frontRight, frontLeft}; 
        // eventual order: return new VisionCameraIo[] {frontRight, back, frontLeft, front}; 
    }
}