package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.manager.ServoMotorSubsystem;

public class Wrist extends ServoMotorSubsystem {

    private ArmFeedforward m_feedforward;

    public Wrist(ServoMotorSubsystemConstants constants) {
        super(constants);

        m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
    }

    @Override
    public void outputTelemetry() {

    }

 @Override
    public void holdPosition() {
        m_pidController.setReference(m_currentState.getPosition(), ControlType.kPosition, m_constants.kDefaultSlot, m_feedforward.calculate(Math.toRadians(m_encoder.getPosition()), Math.toRadians(m_encoder.getVelocity())), ArbFFUnits.kVoltage);
    }

    @Override
    public void runToSetpoint() {
        m_setpoint = m_profile.calculate(Timer.getFPGATimestamp() - m_profileStartTime, new TrapezoidProfile.State(m_desiredState.getPosition(), 0), new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity()));

        m_pidController.setReference(m_setpoint.position, ControlType.kPosition, m_constants.kDefaultSlot, m_feedforward.calculate(m_setpoint.position, m_setpoint.velocity), ArbFFUnits.kVoltage);
    }
    @Override
    public SubsystemType getSubsystemType() {
        return SubsystemType.WRIST;
    }

    public enum WristState implements SubsystemState {
        MANUAL(0, "Manual"),
        HOME(0, "Home"),
        OUT(100, "Out"),
        IN(20, "In");

        private double position;
        private String name;

        private WristState(double position, String name) {
            this.position = position;
            this.name = name;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void setPosition(double position) {
            this.position = position;
        }

        @Override
        public String getName() {
            return name;
        }

    }
    
}
