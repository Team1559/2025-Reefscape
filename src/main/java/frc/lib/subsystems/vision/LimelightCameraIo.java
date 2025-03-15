package frc.lib.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class LimelightCameraIo extends VisionCameraIo {
    private static final double STDEV_MULTIPLIER = 1.5;
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
            // inputs.pose = POSE2D_NAN;
            // inputs.timestamp = Double.NaN;
            inputs.hasPose = false;

            // inputs.stdevX = Double.NaN;
            // inputs.stdevY = Double.NaN;
            // inputs.stdevRotation = ROTATION2D_NAN;
        } else {
            inputs.pose = estimate.pose;
            inputs.timestamp = estimate.timestampSeconds;
            inputs.hasPose = true;
            double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(hostName, "stddevs");
            Logger.recordOutput(getOutputLogPath("stddevs"), stddevs);
            if (usingMegaTag2) {
                // TODO: u pickin up what im puttin down? (Dont do small bc megatag 2 no good
                // with yaw)
                inputs.stdevRotation = MEGATAG2_STDEV_YAW;//Rotation2d.fromDegrees(stddevs[11] * STDEV_MULTIPLIER);
                inputs.stdevX = stddevs[6] * STDEV_MULTIPLIER;
                inputs.stdevY = stddevs[7] * STDEV_MULTIPLIER;
            } else {
                inputs.stdevRotation = Rotation2d.fromDegrees(stddevs[5] * STDEV_MULTIPLIER);
                inputs.stdevX = stddevs[0] * STDEV_MULTIPLIER;
                inputs.stdevY = stddevs[1] * STDEV_MULTIPLIER;
            }

            if (!Double.isFinite(inputs.stdevRotation.getRadians()) || !Double.isFinite(inputs.stdevX)
                    || !Double.isFinite(inputs.stdevY) || !Double.isFinite(inputs.pose.getX())
                    || !Double.isFinite(inputs.pose.getY()) || !Double.isFinite(inputs.pose.getRotation().getRadians())
                    || !Double.isFinite(inputs.timestamp)) {
                inputs.hasPose = false;
            }
        }
    }
}
