package frc.lib.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.BaseIo;

public class VisionCameraIo extends BaseIo<VisionCameraIo.VisionInputs> {
    @AutoLog
    public static abstract class VisionInputs implements LoggableInputs {
        public Pose2d pose;
        public double timestamp;
    }

    public VisionCameraIo(String logPath){
        super(logPath, new VisionInputsAutoLogged());
    }

    protected void updateInputs(VisionInputs inputs) {
        throw new UnsupportedOperationException("Update Inputs");
    }
}
