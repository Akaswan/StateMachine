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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ServoMotorSubsystem extends SubsystemBase {

  protected final ServoMotorSubsystemConstants m_constants;
  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected SubsystemState m_currentState = null;
  protected SubsystemState m_desiredState = null;

  protected final SparkMaxPIDController m_pidController;

  protected TrapezoidProfile m_profile;
  protected TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  protected TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  
  protected double m_profileStartTime = -1;

  protected double m_simPosition;

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

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(m_constants.kMaxVelocity, m_constants.kMaxAcceleration));

    m_currentState = m_constants.kinitialState;
    m_desiredState = m_constants.kinitialState;
  }

  public abstract void outputTelemetry();

  public abstract void runToSetpoint();

  public abstract void holdPosition();

  public abstract SubsystemType getSubsystemType();

  public void manualControl(double throttle, double multiplier, SubsystemState manualState) {
    m_desiredState = manualState;
    m_currentState = manualState;

    manualState.setPosition(m_desiredState.getPosition() + throttle * multiplier);
  }

  public void setState(SubsystemState desiredState) {
    m_desiredState = desiredState;

    m_profileStartTime = Timer.getFPGATimestamp();
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getPosition() - m_encoder.getPosition()) < m_constants.kSetpointTolerance;
  }

  public double getPosition() {
    return RobotBase.isReal() ? m_encoder.getPosition() : m_simPosition;
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_encoder.getVelocity() : 0;
  }

  @Override
  public void periodic() {
    outputTelemetry();


    if (m_profileStartTime == -1) {
      holdPosition();
    } else {
      runToSetpoint();
    }
    
    if (atSetpoint() && m_currentState != m_desiredState) {
      m_currentState = m_desiredState;
      m_profileStartTime = -1;
    }

  }

  public static class ServoMotorSubsystemConstants {
    public String kName = "ERROR_ASSIGN_A_NAME";

    public CANSparkMaxConstants kMasterConstants = new CANSparkMaxConstants();
    public CANSparkMaxConstants[] kSlaveConstants = new CANSparkMaxConstants[0];

    public double kHomePosition = 0.0;
    public double kRotationsPerUnitDistance = 1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1

    // PID Constants
    public double kKp = 0.0;
    public double kKi = 0.0;
    public double kKd = 0.0;

    public double kSetpointTolerance = 0.0; // Tolerance for atSetpoint() 
    public double kSmartMotionTolerance = 0.0; // Tolerance for pid smart motion (Stops ocilation)

    public int kDefaultSlot = 0; // PID Slot, make more if more than one set of pid constants are used

    public double kMaxVelocity = 0.0; // Max velocity for smart motion
    public double kMaxAcceleration = 0.0; // Max acceleration for smart motion

    // Feedforward constants
    public double kKs = 0.0;
    public double kKg = 0.0;
    public double kKv = 0.0;
    public double kKa = 0.0;

    // Max/Min positions the subsystem should be able to move
    public double kMaxPosition = Double.POSITIVE_INFINITY;
    public double kMinPosition = Double.NEGATIVE_INFINITY;

    public SubsystemState kinitialState = null;
  }

  public static class CANSparkMaxConstants {
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
  }

  public interface SubsystemState {
    double getPosition();

    String getName();

    void setPosition(double position);
  }

  public enum SubsystemType {
    ARM,
    ELEVATOR,
    WRIST
  }

}
