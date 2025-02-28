package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystems.LoggableIo;

public class IntakeIo extends LoggableIo<IntakeIo.IntakeInputs> {
    @AutoLog
    public static abstract class IntakeInputs implements LoggableInputs {
        public Rotation2d currentAngle;
        public Rotation2d currentAngleFromMotor;
        public double angleMotorCurrent;
        
    }

    public IntakeIo(String name) {
        super(name, new IntakeInputsAutoLogged());
    }

    public void run(boolean forward) {
        if (forward) {
            Logger.recordOutput(getOutputLogPath("State"), "Forward");
        } else {
            Logger.recordOutput(getOutputLogPath("State"), "Reverse");
        }
    }

    public void setAngle(Rotation2d targetAngle) {
        Logger.recordOutput(getOutputLogPath("TargetAngle"), targetAngle);
    }

    public void stop() {
        Logger.recordOutput(getOutputLogPath("State"), "Stop");
    }
}
