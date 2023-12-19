// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manager;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.feedforward.ArmFeedforward;
import frc.robot.util.feedforward.ElevatorFeedforward;
import frc.robot.util.feedforward.Feedforward;

public abstract class ServoMotorSubsystem extends SubsystemBase {

  protected final ServoMotorSubsystemConstants m_constants;
  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected SubsystemState m_currentState = null;
  protected SubsystemState m_desiredState = null;

  protected final Feedforward m_feedforward;

  protected final ProfiledPIDController m_pidController;

  protected ServoMotorSubsystem(final ServoMotorSubsystemConstants constants) {
    m_constants = constants;

    m_master = new CANSparkMax(m_constants.kMasterConstants.kID, m_constants.kMasterConstants.kMotorType);
    m_master.setIdleMode(m_constants.kMasterConstants.kIdleMode);
    m_master.setSmartCurrentLimit(m_constants.kMasterConstants.kCurrentLimit);

    m_encoder = m_master.getEncoder();

    m_slaves = new CANSparkMax[m_constants.kSlaveConstants.length];

    for (int i = 0; i < m_constants.kSlaveConstants.length; i++) {
      m_slaves[i] = new CANSparkMax(m_constants.kSlaveConstants[i].kID, m_constants.kSlaveConstants[i].kMotorType);
      m_slaves[i].setIdleMode(m_constants.kSlaveConstants[i].kIdleMode);
      m_slaves[i].setSmartCurrentLimit(m_constants.kSlaveConstants[i].kCurrentLimit);
      m_slaves[i].follow(m_master);
    }

    m_pidController = new ProfiledPIDController(m_constants.kKp, m_constants.kKi, m_constants.kKd, new TrapezoidProfile.Constraints(m_constants.kMaxVelocity, m_constants.kMaxAcceleration));

    switch (m_constants.kFeedForwardType) {
      case ARM:
        m_feedforward = new ArmFeedforward(m_constants.kKs, m_constants.kKg, m_constants.kKv, m_constants.kKa);
        break;
      case ELEVATOR:
        m_feedforward = new ElevatorFeedforward(m_constants.kKs, m_constants.kKg, m_constants.kKv, m_constants.kKa);
        break;
      case NONE:
        m_feedforward = null;
        break;
      default:
        m_feedforward = null;
        break;
    }

    m_currentState = m_constants.kinitialState;
    m_currentState = m_constants.kinitialState;
  }

  public abstract void outputTelemetry();

  public synchronized void runToSetpoint() {
    if (m_constants.kFeedForwardType != FeedForwardType.NONE) {
      m_master.set(m_pidController.calculate(m_encoder.getPosition(), m_desiredState.getPosition()));
    }
    
  }

  public enum FeedForwardType {
    ELEVATOR,
    ARM,
    NONE
  }

  public static class ServoMotorSubsystemConstants {
    public String kName = "ERROR_ASSIGN_A_NAME";

    public CANSparkMaxConstants kMasterConstants = new CANSparkMaxConstants();
    public CANSparkMaxConstants[] kSlaveConstants = new CANSparkMaxConstants[0];

    public double kHomePosition = 0.0;
    public double kRotationsPerUnitDistance = 1.0;

    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;
    public double kTolerance = 0.0;

    public double kMaxVelocity = 0.0;
    public double kMaxAcceleration = 0.0;

    public double kKs = 0.0;
    public double kKg = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;

    public double kMaxPosition = Double.POSITIVE_INFINITY;
    public double kMinPosition = Double.NEGATIVE_INFINITY;

    public FeedForwardType kFeedForwardType = FeedForwardType.NONE;

    public SubsystemState kinitialState = null;
  }

  public static class CANSparkMaxConstants {
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit;
  }

  public interface SubsystemState {
    double getPosition();

    String getNames();
  }

}
