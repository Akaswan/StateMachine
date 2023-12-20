package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.manager.ServoMotorSubsystem;

public class Arm extends ServoMotorSubsystem {

    private ArmFeedforward m_feedforward;

    public Arm(ServoMotorSubsystemConstants constants) {
        super(constants);

        m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Desired Position", m_setpoint.position);
        SmartDashboard.putNumber("Position", m_simPosition);
    }

    @Override
    public void holdPosition() {
        m_pidController.setReference(m_currentState.getPosition(), ControlType.kPosition, m_constants.kDefaultSlot, m_feedforward.calculate(Math.toRadians(getPosition()), Math.toRadians(getVelocity())), ArbFFUnits.kVoltage);
    }

    @Override
    public void runToSetpoint() {
        m_setpoint = m_profile.calculate(Timer.getFPGATimestamp() - m_profileStartTime, new TrapezoidProfile.State(m_desiredState.getPosition(), 0), new TrapezoidProfile.State(m_setpoint.position, m_setpoint.velocity));

        if (RobotBase.isReal()) {
            m_pidController.setReference(m_setpoint.position, ControlType.kPosition, m_constants.kDefaultSlot, m_feedforward.calculate(Math.toRadians(m_setpoint.position), Math.toRadians(m_setpoint.velocity)), ArbFFUnits.kVoltage);
        } else {
            m_simPosition = m_setpoint.position;
        }

        // if (m_setpoint.position == m_desiredState.getPosition()) m_profileStartTime = -1;
    }

    @Override
    public SubsystemType getSubsystemType() {
        return SubsystemType.ARM;
    }

    public enum ArmState implements SubsystemState {
        MANUAL(0, "Manual"),
        HOME(0, "Home"),
        OUT(100, "Out"),
        IN(20, "In");

        private double position;
        private String name;

        private ArmState(double position, String name) {
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
