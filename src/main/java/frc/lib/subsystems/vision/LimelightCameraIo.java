package frc.lib.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightCameraIo extends VisionCameraIo {
    private final String name;
    private final Supplier<Rotation2d> yaw;

    public LimelightCameraIo(String logPath, String name, Supplier<Rotation2d> yaw) {
        super(logPath);
        this.name = name;
        this.yaw = yaw;
    }

    @Override
    protected void updateInputs(VisionInputs inputs) {
        LimelightHelpers.SetRobotOrientation(name,
                yaw.get().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate estimate;
        if (DriverStation.isEnabled()) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        }

        if (estimate == null || estimate.tagCount <= 0) {
            inputs.pose = null;
            inputs.timestamp = Double.NaN;
            inputs.hasPose = false;
        } else {
            inputs.pose = estimate.pose;
            inputs.timestamp = estimate.timestampSeconds;
            inputs.hasPose = true;
        }
    }
}
