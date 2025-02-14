package frc.lib.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.lib.LoggableIo;

public class ElevatorIo extends LoggableIo<ElevatorIo.ElevatorInputs> {
    @AutoLog
    public static abstract class ElevatorInputs implements LoggableInputs {
        public boolean isHome;
        public double currentPosition;
        public double motorCurrent;
    }

    public ElevatorIo(String name) {
        super(name, new ElevatorInputsAutoLogged());
    }

    protected void updateInputs(ElevatorInputs inputs) {
        throw new UnsupportedOperationException("Update Inputs");
    }

    public void setTargetPosition(double pos) {
        Logger.recordOutput(getOutputLogPath("TargetPosition"), pos);
    }

    public void stop() {
        double currentPos = getInputs().currentPosition;
        setTargetPosition(currentPos);
    }

    public void goToZero(){
        Logger.recordOutput(getOutputLogPath("TargetPosition"), 0);
    }
}