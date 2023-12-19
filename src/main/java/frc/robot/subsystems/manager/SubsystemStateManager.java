package frc.robot.subsystems.manager;

public class SubsystemStateManager {

    private SuperstructureState m_currentState;
    private SuperstructureState m_desiredState;

    // Declare subsystems here
    


    // Example states
    public enum SuperstructureState {
        STOW,
        PICKUP,
        PLACE
    }

    public SubsystemStateManager(SuperstructureState initialState) {
        m_currentState = initialState;
        m_desiredState = initialState;
    }

    public void setDesiredState(SuperstructureState desiredState) {
        m_desiredState = desiredState;
    }

    
    
}
