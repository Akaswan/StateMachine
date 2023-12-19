package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.subsystems.manager.ServoMotorSubsystem;

public class Arm extends ServoMotorSubsystem {

    private ArmFeedforward m_feedforward;

    protected Arm(ServoMotorSubsystemConstants constants) {
        super(constants);

        m_feedforward = new ArmFeedforward(constants.kKs, constants.kKg, constants.kKv, constants.kKa);
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void runToSetpoint() {
        m_pidController.setReference(m_desiredState.getPosition(), ControlType.kSmartMotion, m_constants.kDefaultSlot, m_feedforward.calculate(0, 0, 0), ArbFFUnits.kVoltage);
    }
    
}
