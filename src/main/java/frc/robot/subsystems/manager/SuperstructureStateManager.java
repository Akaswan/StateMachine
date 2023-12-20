package frc.robot.subsystems.manager;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetSubsystemState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Wrist.WristState;

public class SuperstructureStateManager {

    private SuperstructureState m_currentState;
    private SuperstructureState m_desiredState;

    // Declare subsystems here


    // Example states
    public enum SuperstructureState {
        HOME(new SubsystemStateContainer(ArmState.HOME, WristState.HOME, ElevatorState.HOME), "Home"),
        PICKUP(new SubsystemStateContainer(ArmState.IN, WristState.IN, ElevatorState.IN), "Pickup"),
        PLACE(new SubsystemStateContainer(ArmState.OUT, WristState.OUT, ElevatorState.OUT), "Place");

        SubsystemStateContainer subsystemStates;
        String name;

        private SuperstructureState(SubsystemStateContainer subsystemStates, String name) {
            this.subsystemStates = subsystemStates;
            this.name = name;
        }

        public SubsystemStateContainer getSubsystemStates() {
            return subsystemStates;
        }

        public String getName() {
            return name;
        }
    }

    public SuperstructureStateManager(SuperstructureState initialState, Arm arm) {
        m_currentState = initialState;
        m_desiredState = initialState;
    }

    public void setDesiredState(SuperstructureState desiredState) {
        m_desiredState = desiredState;
    }

    public static SequentialCommandGroup goToState(int[] order, SuperstructureState desiredState) {
        SequentialCommandGroup goToStateCommand = new SequentialCommandGroup();

        goToStateCommand.addCommands(new SetSubsystemState(null, null));

        return goToStateCommand;
    }

    
    
}
