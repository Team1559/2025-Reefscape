package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.commands.DynamicCommand;

public class DynamicCoralCommand extends DynamicCommand{

    public DynamicCoralCommand(){

    }

    @Override
    public Command getCommand(){
        return new PrintCommand("TD");
    }

}