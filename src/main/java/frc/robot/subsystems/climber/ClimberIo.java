package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.subsystems.LoggableIo;

public class ClimberIo extends LoggableIo<ClimberIo.ClimberInputs> {
    @AutoLog
    public static abstract class ClimberInputs implements LoggableInputs {
        public boolean isDone;
        public double motorCurrent;
    }
    public ClimberIo(String name) {
        super(name, new ClimberInputsAutoLogged());
    }

    public void run() {
        Logger.recordOutput(getOutputLogPath("IsRunning"), true);
    }
    public void stop() {
        Logger.recordOutput(getOutputLogPath("IsRunning"), false);
    }
}
