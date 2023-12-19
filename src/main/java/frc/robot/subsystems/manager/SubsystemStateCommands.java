package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manager.SubsystemStateManager.SuperstructureState;

public class SubsystemStateCommands {

    public static Command scoreCommand(int[] order, SuperstructureState desiredState) {
        Command scoreCommand = new Command() {};

        scoreCommand.andThen(new SetSubsystemState())

        return scoreCommand;
    }
    
}
