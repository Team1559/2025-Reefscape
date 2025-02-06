package frc.lib.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightCameraIo extends VisionCameraIo {
    private final String hostName;
    private final Supplier<Rotation2d> yaw;

    public LimelightCameraIo(String ioName, String hostName, Supplier<Rotation2d> yaw) {
        super(ioName);
        this.yaw = yaw;
        this.hostName = hostName;
    }

    @Override
    protected void updateInputs(VisionInputs inputs) {
        LimelightHelpers.SetRobotOrientation(hostName,
                yaw.get().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate estimate;
        if (DriverStation.isEnabled()) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(hostName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(hostName);
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
