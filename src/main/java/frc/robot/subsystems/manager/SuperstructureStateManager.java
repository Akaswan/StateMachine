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

    private SubsystemStateContainer m_subsystems;

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

        public SubsystemState getArmState() {
            return subsystemStates.getArmState()
        }

        public SubsystemState getElevatorState() {
            return subsystemStates.getElevatorState()
        }

        public SubsystemState getWristState() {
            return subsystemStates.getWristState()
        }

        public String getName() {
            return name;
        }
    }

    public SuperstructureStateManager(SuperstructureState initialState) {
        m_currentState = initialState;
        m_desiredState = initialState;
    }

    public void setDesiredState(SuperstructureState desiredState) {
        m_desiredState = desiredState;
    }

    public SequentialCommandGroup goToState(ServoMotorSubsystem[] order, SuperstructureState desiredState) {
        SequentialCommandGroup goToStateCommand = new SequentialCommandGroup();

        for (int i = 0; i < order.length; i++) {
            goToStateCommand.addCommands(new SetSubsystemState(order[i], desiredState))
        }
        

        return goToStateCommand;
    }

    
    
}
