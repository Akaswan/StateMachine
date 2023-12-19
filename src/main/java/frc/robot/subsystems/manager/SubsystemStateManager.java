package frc.robot.subsystems.manager;

public class SubsystemStateManager {

    private SuperstructureState m_currentState;
    private SuperstructureState m_desiredState;

    // Declare subsystems here
    private Arm m_arm;


    // Example states
    public enum SuperstructureState {
        HOME({ArmState.HOME}, ),
        PICKUP,
        PLACE;

        SubsystemState[] subsystemStates;
        double String name;

        public SuperstructureState(SubsystemState[] subsystemStates, String name) {
            this.subsystemStates = subsystemStates;
            this.name = name;
        }

        public SubsystemState[] getSubsystemStates() {
            return subsystemStates;
        }

        public getName() {
            return name;
        }
    }

    public SubsystemStateManager(SuperstructureState initialState, Arm arm) {
        m_currentState = initialState;
        m_desiredState = initialState;
        m_arm = arm;
    }

    public void setDesiredState(SuperstructureState desiredState) {
        m_desiredState = desiredState;
    }

    
    
}
