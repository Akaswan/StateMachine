// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manager;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ServoMotorSubsystem extends SubsystemBase {

  protected final ServoMotorSubsystemConstants m_constants;
  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected SubsystemState m_currentState = null;
  protected SubsystemState m_desiredState = null;

  protected final SparkMaxPIDController m_pidController;

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

    m_pidController = m_master.getPIDController();
    m_pidController.setP(m_constants.kKp, m_constants.kDefaultSlot);
    m_pidController.setI(m_constants.kKi, m_constants.kDefaultSlot);
    m_pidController.setD(m_constants.kKd, m_constants.kDefaultSlot);
    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, m_constants.kDefaultSlot);
    m_pidController.setSmartMotionMaxAccel(m_constants.kMaxAcceleration, m_constants.kDefaultSlot);
    m_pidController.setSmartMotionMaxVelocity(m_constants.kMaxVelocity, m_constants.kDefaultSlot);

    m_currentState = m_constants.kinitialState;
    m_currentState = m_constants.kinitialState;
  }

  public abstract void outputTelemetry();

  public abstract void runToSetpoint();

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

    public int kDefaultSlot = 0;

    public double kMaxVelocity = 0.0;
    public double kMaxAcceleration = 0.0;

    public double kKs = 0.0;
    public double kKg = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;

    public double kMaxPosition = Double.POSITIVE_INFINITY;
    public double kMinPosition = Double.NEGATIVE_INFINITY;

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
