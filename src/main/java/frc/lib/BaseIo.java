package frc.lib;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class BaseIo<T extends LoggableInputs> {
    private final String logPath;
    private final T inputs;

    protected BaseIo(String logPath, T inputs){
        this.logPath = logPath;
        this.inputs = inputs;
    }

    public final T getInputs(){
        return inputs;
    }

    public final void periodic(){
        updateInputs(inputs); //FIXME: NOT ON REPLAY!
        Logger.processInputs(logPath, inputs);
    }

    protected final String getLogPath(String suffix){
        return logPath + "/" + suffix;
    }

    protected abstract void updateInputs(T inputs);

}