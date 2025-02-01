package frc.lib.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.elevator.ElevatorIo.ElevatorInputs;

public class Elevator extends SubsystemBase {
    private ElevatorIo io;
    private double targetPosition;

    public Elevator(ElevatorIo io) {
        this.io = io;
        targetPosition = 0.0;
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
        io.setTargetPosition(pos);
    }
    public boolean isAtTargetPosition (double tolerance) {
        ElevatorInputs inputs = io.getInputs();
        return Math.abs(inputs.currentPosition - targetPosition) < tolerance;
    }
    public void stop() {
        io.stop();
    }
}
