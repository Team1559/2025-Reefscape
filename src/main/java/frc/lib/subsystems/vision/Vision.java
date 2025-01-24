package frc.lib.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.vision.VisionCameraIo.VisionInputs;

public class Vision extends SubsystemBase {
    private final VisionCameraIo[] cameras;
    private final VisionConsumer visionConsumer;

    public Vision(VisionConsumer visionConsumer, VisionCameraIo... cameras) {
        this.visionConsumer = visionConsumer;
        this.cameras = cameras;
    }

    public void periodic() {
        for (VisionCameraIo cam : cameras){
            cam.periodic();
            VisionInputs inputs = cam.getInputs();
            if (inputs.pose != null){
                visionConsumer.addVisionMeasurement(inputs.pose, inputs.timestamp, VecBuilder.fill(0,0,0)); //FIXME: add standard deviations
            }
        }
    }
}
