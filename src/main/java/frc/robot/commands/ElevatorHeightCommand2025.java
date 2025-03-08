package frc.robot.commands;

import frc.lib.commands.ElevatorHeightCommand;
import frc.lib.subsystems.elevator.Elevator;
import frc.robot.subsystems.Elevator2025.Level;

public class ElevatorHeightCommand2025 extends ElevatorHeightCommand {
    private static final double TOLERANCE = 0.002;

    public ElevatorHeightCommand2025(Elevator elevator, Level targetLevel) {
        super(elevator, targetLevel.height, TOLERANCE);
    }
}
