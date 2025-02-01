package frc.lib.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.BaseIo;

public class ElevatorIo extends BaseIo<ElevatorIo.ElevatorInputs> {
    @AutoLog
    public static abstract class ElevatorInputs implements LoggableInputs {
        public boolean lowerLimitSwitch;
        public double currentPosition;
        public double motorCurrent;
    }

    public ElevatorIo(String logPath) {
        super(logPath, new ElevatorInputsAutoLogged());
    }

    protected void updateInputs(ElevatorInputs inputs) {
        throw new UnsupportedOperationException("Update Inputs");
    }

    public void setTargetPosition(double pos) {
        Logger.recordOutput(getLogPath("TargetPosition"), pos);
    }

    public void stop() {
        double currentPos = getInputs().currentPosition;
        setTargetPosition(currentPos);
    }
}