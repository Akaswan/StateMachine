// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manager;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ServoMotorSubsystem extends SubsystemBase {

  protected final ServoMotorSubsystemConstants m_constants;
  protected final CANSparkMax m_master;
  protected final CANSparkMax[] m_slaves;
  protected final RelativeEncoder m_encoder;

  protected SubsystemState m_lastHeldState = null;
  protected SubsystemState m_currentState = null;
  protected SubsystemState m_desiredState = null;
  protected SubsystemState m_previousDesiredState = null;

  protected final SparkMaxPIDController m_pidController;

  protected TrapezoidProfile m_profile;
  protected TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  protected TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  
  protected double m_profileStartTime = -1;

  protected double m_arbFeedforward = 0;

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

    m_lastHeldState = m_constants.kInitialState;
    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;
    m_previousDesiredState = m_constants.kInitialState;
  }

  public abstract void outputTelemetry();

  public abstract SubsystemType getSubsystemType();

  public abstract void subsystemPeriodic();

  public SubsystemState getCurrentState() {
    return m_currentState;
  }

  public void manualControl(double throttle, double multiplier, double deadband) {
    double m_throttle = MathUtil.applyDeadband(throttle, deadband);

    if (m_currentState != m_constants.kManualState) m_constants.kManualState.setPosition(getPosition());

    if (Math.abs(m_throttle) > 0 && m_profileStartTime == -1) {
      m_lastHeldState = m_constants.kManualState;
      m_desiredState = m_constants.kManualState; 
      m_currentState = m_constants.kManualState;

      m_throttle *= multiplier;

      if (m_constants.kManualState.getPosition() + m_throttle >= m_constants.kMinPosition && m_constants.kManualState.getPosition() + m_throttle < m_constants.kMaxPosition || 
          m_constants.kManualState.getPosition() + m_throttle <= m_constants.kMaxPosition && m_constants.kManualState.getPosition() + m_throttle > m_constants.kMinPosition) {
            m_constants.kManualState.setPosition(m_constants.kManualState.getPosition() + m_throttle);
      } else if (m_constants.kManualState.getPosition() <= m_constants.kMinPosition) {
        m_constants.kManualState.setPosition(m_constants.kMinPosition);
      } else if (m_constants.kManualState.getPosition() >= m_constants.kMaxPosition){
        m_constants.kManualState.setPosition(m_constants.kMaxPosition);
      }
    } 
  }

  public void holdPosition() {
    m_pidController.setReference(m_currentState.getPosition(), ControlType.kPosition, m_constants.kDefaultSlot, m_arbFeedforward, ArbFFUnits.kVoltage);
  }

  public void runToSetpoint() {
      if (m_previousDesiredState != m_desiredState || m_lastHeldState == m_constants.kManualState) {
        if (m_lastHeldState == m_constants.kManualState) {
          m_constants.kSetpointSwitchState.setPosition(m_constants.kManualState.getPosition());
          m_constants.kSetpointSwitchState.setVelocity(m_constants.kManualState.getVelocity());
          m_lastHeldState = m_constants.kSetpointSwitchState;
        } else {
          m_constants.kSetpointSwitchState.setPosition(m_setpoint.position);
          m_constants.kSetpointSwitchState.setVelocity(m_setpoint.velocity);
          m_lastHeldState = m_constants.kSetpointSwitchState;
        }
      }

      m_setpoint = m_profile.calculate(Timer.getFPGATimestamp() - m_profileStartTime, new TrapezoidProfile.State(m_desiredState.getPosition(), 0), new TrapezoidProfile.State(m_lastHeldState.getPosition(), m_lastHeldState.getVelocity()));
     
      m_pidController.setReference(m_setpoint.position, ControlType.kPosition, m_constants.kDefaultSlot, m_arbFeedforward, ArbFFUnits.kVoltage);

      if (m_currentState != m_constants.kTransitionState) m_currentState = m_constants.kTransitionState;

      m_constants.kTransitionState.setPosition(m_setpoint.position);
      m_constants.kTransitionState.setVelocity(m_setpoint.velocity);

      if (m_setpoint.position == m_desiredState.getPosition()) {
        m_profileStartTime = -1;
        m_lastHeldState = m_desiredState;
        m_currentState = m_desiredState;
        m_constants.kManualState.setPosition(0); 
      } 

      m_previousDesiredState = m_desiredState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public void setState(SubsystemState desiredState) {
    m_desiredState = desiredState;
    m_profileStartTime = Timer.getFPGATimestamp();
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getPosition() - getPosition()) < m_constants.kSetpointTolerance;
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public double getVelocity() {
    return RobotBase.isReal() ? m_encoder.getVelocity() : 0;
  }

  public ServoMotorSubsystemConstants getConstants() {
    return m_constants;
  }

  @Override
  public void periodic() {
    subsystemPeriodic();

    if (m_profileStartTime == -1) {
      holdPosition();
    } else {
      runToSetpoint();
    }
  
    outputTelemetry();
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

    // Manual constants
    public int kManualAxis = 0;
    public double kManualMultiplier = 0;
    public double kManualDeadZone = 0;

    public SubsystemState kInitialState = null;
    public SubsystemState kManualState = null;
    public SubsystemState kTransitionState = null;
    public SubsystemState kSetpointSwitchState = null;
  }

  public static class CANSparkMaxConstants {
    public int kID = 0;
    public IdleMode kIdleMode = IdleMode.kBrake;
    public MotorType kMotorType = MotorType.kBrushless;
    public int kCurrentLimit = 0;
  }

  public interface SubsystemState {
    double getPosition();

    double getVelocity();

    void setPosition(double position);

    void setVelocity(double velocity);

    String getName();    
  }

  public enum SubsystemType {
    ARM,
    ELEVATOR,
    WRIST
  }

}
