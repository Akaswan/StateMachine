package frc.lib.templates.motors;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.templates.commands.ClosedLoopTrapezoidProfileCommand;
import frc.lib.templates.subsystems.PositionSubsystem;
import frc.lib.templates.subsystems.PositionSubsystem.SubsystemState;
import frc.lib.templates.subsystems.SubsystemConstants.MotorConstants;
import java.util.function.Supplier;

public class SubsystemSparkMax implements SubsystemMotor {

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkPIDController m_pidController;

  private Supplier<SubsystemState> m_desiredState;
  private Supplier<Double> m_feedForward;

  private MotorConstants m_constants;

  @Override
  public void configureMotor(
      MotorConstants constants,
      Supplier<SubsystemState> desiredState,
      Supplier<Double> feedForward) {
    m_constants = constants;

    m_desiredState = desiredState;
    m_feedForward = feedForward;

    m_motor = new CANSparkMax(m_constants.kID, m_constants.kMotorType);
    m_encoder = m_motor.getEncoder();
    m_pidController = m_motor.getPIDController();

    m_motor.setIdleMode(m_constants.kIdleMode);
    m_motor.setSmartCurrentLimit(m_constants.kCurrentLimit);
    m_motor.setInverted(m_constants.kInverted);

    m_pidController.setP(m_constants.kKp);
    m_pidController.setI(m_constants.kKi);
    m_pidController.setD(m_constants.kKd);

    m_encoder.setPosition(m_constants.kHomePosition);
    m_encoder.setPositionConversionFactor(m_constants.kPositionConversionFactor);
    m_encoder.setVelocityConversionFactor(m_constants.kPositionConversionFactor / 60);

    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);

    m_motor.burnFlash();
  }

  @Override
  public double getPostion() {
    return m_encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  @Override
  public void runToPosition() {
    m_pidController.setReference(
        m_desiredState.get().getPosition(),
        ControlType.kPosition,
        0,
        m_feedForward.get(),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void runToVelocity() {
    m_pidController.setReference(
        m_desiredState.get().getVelocity(),
        ControlType.kVelocity,
        0,
        m_feedForward.get(),
        ArbFFUnits.kVoltage);
  }

  @Override
  public Command runToPositionWithProfile(
      TrapezoidProfile profile,
      SubsystemState transitionState,
      Supplier<TrapezoidProfile.State> profileStartSetpoint,
      SubsystemState desiredState,
      Supplier<Boolean> isAtSetpoint,
      Runnable preCommand,
      BooleanConsumer postCommand,
      PositionSubsystem subsystem) {
    return new ClosedLoopTrapezoidProfileCommand(
            profile,
            (TrapezoidProfile.State state) -> {
              transitionState.setSetpoint(state);
              runToPosition();
            },
            () -> desiredState.getSetpoint(),
            profileStartSetpoint,
            isAtSetpoint,
            subsystem)
        .beforeStarting(preCommand)
        .finallyDo(postCommand);
  }
}
