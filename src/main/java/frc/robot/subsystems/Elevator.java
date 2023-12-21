package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.manager.ServoMotorSubsystem;

public class Elevator extends ServoMotorSubsystem {

    private ElevatorFeedforward m_feedforward;

    public Elevator(ServoMotorSubsystemConstants constants) {
        super(constants);

        m_feedforward = new ElevatorFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putNumber("Desired Position", m_desiredState.getPosition());
        // SmartDashboard.putNumber("Current Position", m_currentState.getPosition());
        // SmartDashboard.putNumber("Last Held Position", m_lastHeldState.getPosition());
        // SmartDashboard.putNumber("Last Held Velocity", m_lastHeldState.getVelocity());
        // SmartDashboard.putString("Last Held State", m_lastHeldState.getName());
        // SmartDashboard.putString("Current State", m_currentState.getName());
        // SmartDashboard.putNumber("Current State Velocity", m_currentState.getVelocity());
        // SmartDashboard.putNumber("Current setpoint Velocity", m_setpoint.velocity);
        // SmartDashboard.putString("Desired State", m_desiredState.getName());
    }

    @Override
    public void subsystemPeriodic() {
        setFeedforward(m_feedforward.calculate(m_encoder.getPosition(), m_encoder.getVelocity()));

        Arm.root.setPosition(1.5, m_simPosition);
    }


    @Override
    public SubsystemType getSubsystemType() {
        return SubsystemType.ELEVATOR;
    }

    public enum ElevatorState implements SubsystemState {
        MANUAL(0, 0, "Manual"),
        TRANSITION(0, 0, "Transition"),
        SETPOINT_SWITCH(0, 0, "Setpoint Switch"),
        HOME(0, 0, "Home"),
        OUT(1.5, 0, "Out"),
        IN(.5, 0, "In");

        private double position;
        private double velocity;
        private String name;

        private ElevatorState(double position, double velocity, String name) {
            this.position = position;
            this.velocity = velocity;
            this.name = name;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public void setPosition(double position) {
            this.position = position;
        }

        @Override
        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public String getName() {
            return name;
        }

    }
    
}
