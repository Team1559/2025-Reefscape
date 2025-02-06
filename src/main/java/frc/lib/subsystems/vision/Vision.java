package frc.lib.subsystems.vision;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggableSubsystem;
import frc.lib.subsystems.vision.VisionCameraIo.VisionInputs;

public class Vision extends LoggableSubsystem {
    private final VisionCameraIo[] cameras;
    private final VisionConsumer visionConsumer;

    public Vision(String name, VisionConsumer visionConsumer, VisionCameraIo... cameras) {
        super(name);
        this.visionConsumer = visionConsumer;
        this.cameras = cameras;

        for (VisionCameraIo visionCameraIo : cameras) {
            addIo(visionCameraIo, "Cameras");
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        boolean robotHasPose = false;
        for (VisionCameraIo cam : cameras){
            VisionInputs inputs = cam.getInputs();
            if (inputs.hasPose){
                visionConsumer.addVisionMeasurement(inputs.pose, inputs.timestamp, VecBuilder.fill(0,0,0)); //FIXME: add standard deviations
                robotHasPose = true;
            }
        }
        Logger.recordOutput(getLogPath("HasVisionRead"), robotHasPose);
    }
}
