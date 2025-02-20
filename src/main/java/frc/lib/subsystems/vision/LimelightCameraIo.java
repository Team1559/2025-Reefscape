package frc.lib.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightCameraIo extends VisionCameraIo {
    private static final double MEGATAG2_STDEV_MULTIPLIER = 0.5;
    private static final Rotation2d MEGATAG2_STDEV_YAW = Rotation2d.fromRadians(999999);
    private static final Rotation2d ROTATION2D_NAN = new Rotation2d(Double.NaN);
    private static final Pose2d POSE2D_NAN = new Pose2d(Double.NaN, Double.NaN, ROTATION2D_NAN);
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

        boolean usingMegaTag2 = DriverStation.isEnabled();
        if (usingMegaTag2) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(hostName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(hostName);
        }
        
        if (estimate == null || estimate.tagCount <= 0) {
            inputs.pose = POSE2D_NAN;
            inputs.timestamp = Double.NaN;
            inputs.hasPose = false;
            
            inputs.stdevX = Double.NaN;
            inputs.stdevY = Double.NaN;
            inputs.stdevRotation = ROTATION2D_NAN;
        } else {
            inputs.pose = estimate.pose;
            inputs.timestamp = estimate.timestampSeconds;
            inputs.hasPose = true;
            
            Pose3d posRelativeToCamera = LimelightHelpers.getTargetPose3d_CameraSpace(hostName);
            double distance = posRelativeToCamera.getTranslation().getNorm();
            if (usingMegaTag2) {
                // TODO: u pickin up what im puttin down? (Dont do small bc megatag 2 no good
                // with yaw)
                inputs.stdevRotation = MEGATAG2_STDEV_YAW;
                inputs.stdevX = distance * MEGATAG2_STDEV_MULTIPLIER;
                inputs.stdevY = distance * MEGATAG2_STDEV_MULTIPLIER;
            } else {
                inputs.stdevRotation = Rotation2d.fromRadians(distance);
                inputs.stdevX = distance;
                inputs.stdevY = distance;
            }
        }
    }
}
