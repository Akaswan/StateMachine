package frc.robot.util;

public class MechanismStateManager {

    private SuperstructureState m_currentState;
    private SuperstructureState m_desiredState;

    // Declare subsystems here
    


    // Example states
    public enum SuperstructureState {
        STOW,
        PICKUP,
        PLACE
    }

    public MechanismStateManager(SuperstructureState initialState) {
        m_currentState = initialState;
        m_desiredState = initialState;
    }

    public void setDesiredState(SuperstructureState desiredState) {
        m_desiredState = desiredState;
    }

    
    
}
