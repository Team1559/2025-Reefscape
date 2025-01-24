package frc.lib.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightCamera extends VisionCameraIo {
    private final String name;
    private final Supplier<Rotation2d> yaw;
    public LimelightCamera(String logPath, String name, Supplier<Rotation2d> yaw) {
        super(logPath);
        this.name = name;
        this.yaw = yaw;
    }

    @Override
    protected void updateInputs(VisionInputs inputs) {
        LimelightHelpers.SetRobotOrientation(name,
                yaw.get().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (estimate.tagCount > 0) {
            inputs.pose = estimate.pose;
            inputs.timestamp = estimate.timestampSeconds;
        }
        else {
            inputs.pose = null;
            inputs.timestamp = estimate.timestampSeconds;
        }
    }
}
