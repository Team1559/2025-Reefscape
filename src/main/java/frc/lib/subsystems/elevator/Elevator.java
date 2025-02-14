package frc.lib.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggableSubsystem;
import frc.lib.subsystems.elevator.ElevatorIo.ElevatorInputs;

public class Elevator extends LoggableSubsystem {
    private ElevatorIo io;
    private double targetPosition;

    public Elevator(String name, ElevatorIo io) {
        super(name);
        this.io = io;
        targetPosition = 0.0;

        addIo(io);
    }

    public void setTargetPosition(double pos) {
        targetPosition = pos;
        System.out.println(targetPosition);
        io.setTargetPosition(pos);
    }

    public void changeTargetPosition(double diff){
        setTargetPosition(io.getInputs().currentPosition + diff);
    }

    public boolean isAtTargetPosition (double tolerance) {
        ElevatorInputs inputs = io.getInputs();
        return Math.abs(inputs.currentPosition - targetPosition) < tolerance;
    }

    public void stop() {
        io.stop();
    }

    public void goToZero(){
        io.goToZero(); 
        targetPosition = 0;  
    }
    public boolean isHome(){
        return io.getInputs().isHome;
    }
}
