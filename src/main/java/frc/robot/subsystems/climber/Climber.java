package frc.robot.subsystems.climber;

import frc.lib.subsystems.LoggableSubsystem;

public class Climber extends LoggableSubsystem {
    private final ClimberIo io;

    public Climber(String name, ClimberIo io) {
        super(name);
        this.io = io;
        addIo(io);
    }

    public void run() {
        io.run();
    }

    public void stop() {
        io.stop();
    }

    public void testReverse(){
        io.testReverse();
    }
}
