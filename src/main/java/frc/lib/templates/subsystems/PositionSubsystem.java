// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.templates.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.templates.motors.SubsystemMotor;
import frc.lib.templates.motors.SubsystemSparkMax;
import frc.lib.templates.subsystems.SubsystemConstants.MotorControllerType;
import frc.lib.templates.subsystems.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public abstract class PositionSubsystem extends SubsystemBase {

  public PositionSubsystemConstants m_constants;

  protected final SubsystemMotor m_leader;
  protected final CANSparkBase[] m_followers;

  protected TrapezoidProfile m_profile;
  protected TrapezoidProfile.State m_profileStartSetpoint;

  protected SubsystemState m_currentState = null;
  protected SubsystemState m_desiredState = null;

  protected double m_arbFeedforward = 0;

  protected PositionSubsystem(final PositionSubsystemConstants constants) {

    m_constants = constants;

    m_currentState = m_constants.kInitialState;
    m_desiredState = m_constants.kInitialState;

    switch (m_constants.kLeaderConstants.kMotorControllerType) {
      case SPARK_FLEX:
        m_leader = new SubsystemSparkMax();
        break;
      case SPARK_MAX:
        m_leader = new SubsystemSparkMax();
        break;
      case TALON_FX:
        m_leader = new SubsystemSparkMax();
        break;
      default:
        throw new Error("No Motor Configured For The " + m_constants.kSubsystemName + " Subsystem");
    }

    m_leader.configureMotor(
        constants.kLeaderConstants, this::getDesiredState, this::getFeedForward);

    if (m_constants.kFollowerConstants.length > 0) {
      if (m_constants.kFollowerConstants[0].kMotorControllerType == MotorControllerType.SPARK_MAX) {
        m_followers = new CANSparkMax[m_constants.kFollowerConstants.length];
      } else {
        m_followers = new CANSparkFlex[m_constants.kFollowerConstants.length];
      }
    } else {
      m_followers = new CANSparkBase[0];
    }

    for (int i = 0; i < m_constants.kFollowerConstants.length; i++) {
      if (m_constants.kFollowerConstants[0].kMotorControllerType == MotorControllerType.SPARK_MAX) {
        m_followers[i] =
            new CANSparkMax(
                m_constants.kFollowerConstants[i].kID,
                m_constants.kFollowerConstants[i].kMotorType);
      } else {
        m_followers[i] =
            new CANSparkFlex(
                m_constants.kFollowerConstants[i].kID,
                m_constants.kFollowerConstants[i].kMotorType);
      }

      m_followers[i].setIdleMode(m_constants.kFollowerConstants[i].kIdleMode);
      m_followers[i].setSmartCurrentLimit(m_constants.kFollowerConstants[i].kCurrentLimit);
      // m_followers[i].follow(m_leader, m_constants.kFollowerConstants[i].kInverted);

      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65534);
      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65534);
      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
      m_followers[i].setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);

      m_followers[i].burnFlash();
    }

    m_profile = new TrapezoidProfile(m_constants.kProfileConstraints);

    m_leader.runToPosition();

    setName(m_constants.kSubsystemName);
  }

  public void manualControl() {
    double m_throttle = 0;

    switch (m_constants.kManualControlMode) {
      case BUMPERS:
        m_throttle =
            RobotContainer.m_operatorController.getHID().getLeftBumper()
                ? -1
                : (RobotContainer.m_operatorController.getHID().getRightBumper() ? 1 : 0);
        break;
      case LEFT_X:
        m_throttle = RobotContainer.m_operatorController.getLeftX();
        break;
      case LEFT_Y:
        m_throttle = -RobotContainer.m_operatorController.getLeftY();
        break;
      case RIGHT_X:
        m_throttle = RobotContainer.m_operatorController.getRightX();
        break;
      case RIGHT_Y:
        m_throttle = -RobotContainer.m_operatorController.getRightY();
        break;
      case TRIGGERS:
        m_throttle =
            RobotContainer.m_operatorController.getRightTriggerAxis()
                - RobotContainer.m_operatorController.getLeftTriggerAxis();
        break;
    }

    m_throttle = MathUtil.applyDeadband(m_throttle, m_constants.kManualDeadBand);

    if (Math.abs(m_throttle) > 0) {
      if (m_currentState != BuiltInPositionSubsystemState.MANUAL)
        BuiltInPositionSubsystemState.MANUAL.setPosition(getPosition());

      m_desiredState = BuiltInPositionSubsystemState.MANUAL;
      m_currentState = BuiltInPositionSubsystemState.MANUAL;

      m_throttle *= m_constants.kManualMultiplier;

      double intendedPosition =
          MathUtil.clamp(
              BuiltInPositionSubsystemState.MANUAL.getPosition() + m_throttle,
              m_constants.kMinPosition,
              m_constants.kMaxPosition);

      if (intendedPosition != BuiltInPositionSubsystemState.MANUAL.getPosition()) {
        BuiltInPositionSubsystemState.MANUAL.setPosition(intendedPosition);
        m_leader.runToPosition();
      }
    }
  }

  public SubsystemState getCurrentState() {
    return m_currentState;
  }

  public SubsystemState getDesiredState() {
    return m_desiredState;
  }

  public void setFeedforward(double feedforward) {
    m_arbFeedforward = feedforward;
  }

  public double getFeedForward() {
    return m_arbFeedforward;
  }

  public void setDesiredState(SubsystemState desiredState) {
    m_profileStartSetpoint = m_currentState.getSetpoint();

    if (m_currentState == BuiltInPositionSubsystemState.MANUAL) {
      m_profileStartSetpoint.velocity = 0;
    }

    BuiltInPositionSubsystemState.TRANSITION.setSetpoint(m_profileStartSetpoint);
    m_currentState = BuiltInPositionSubsystemState.TRANSITION;
    m_desiredState = desiredState;
  }

  public Command moveWithProfile(SubsystemState desiredState, PositionSubsystem subsystem) {
    return m_leader.runToPositionWithProfile(
        m_profile,
        BuiltInPositionSubsystemState.TRANSITION,
        () -> m_profileStartSetpoint,
        desiredState,
        this::atSetpoint,
        () -> setDesiredState(desiredState),
        (boolean interrupted) -> {
          if (!interrupted) {
            m_currentState = m_desiredState;
          }
        },
        subsystem);
  }
  
  public double getPosition() {
    return Constants.kCurrentMode == Mode.REAL
        ? m_leader.getPostion()
        : m_currentState.getPosition();
  }

  public double getVelocity() {
    return Constants.kCurrentMode == Mode.REAL
        ? m_leader.getVelocity()
        : m_currentState.getVelocity();
  }

  public boolean atSetpoint() {
    return Math.abs(m_desiredState.getPosition() - m_currentState.getPosition())
        <= m_constants.kSetpointTolerance;
  }

  @Override
  public void periodic() {
    if (Constants.kInfoMode) {
      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Current State",
          m_currentState.getName());
      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Desired State",
          m_desiredState.getName());

      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Current Position",
          getPosition());
      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Desired Position",
          m_desiredState.getPosition());
      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Current Velocity",
          getVelocity());
      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Desired Velocity",
          m_currentState.getVelocity());

      Logger.recordOutput(
          m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/At Setpoint",
          atSetpoint());
    }

    subsystemPeriodic();
  }

  public abstract void subsystemPeriodic();

  public abstract void outputTelemetry();

  public enum BuiltInPositionSubsystemState implements SubsystemState {
    TRANSITION,
    MANUAL;

    private TrapezoidProfile.State state = new TrapezoidProfile.State(0, 0);

    private BuiltInPositionSubsystemState(double position) {
      state.position = position;
    }

    private BuiltInPositionSubsystemState() {}

    @Override
    public String getName() {
      return toString();
    }

    @Override
    public State getSetpoint() {
      return state;
    }

    @Override
    public double getPosition() {
      return state.position;
    }

    @Override
    public double getVelocity() {
      return state.velocity;
    }

    @Override
    public void setSetpoint(State state) {
      this.state = state;
    }

    @Override
    public void setPosition(double position) {
      state.position = position;
    }

    @Override
    public void setVelocity(double velocity) {
      state.velocity = velocity;
    }

    @Override
    public double getVoltage() {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
    }

    @Override
    public void setVoltage(double voltage) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }
  }

  public interface SubsystemState {
    String getName();

    TrapezoidProfile.State getSetpoint();

    void setSetpoint(TrapezoidProfile.State state);

    double getPosition();

    void setPosition(double position);

    double getVelocity();

    void setVelocity(double velocity);

    double getVoltage();

    void setVoltage(double voltage);
  }
}

// EXAMPLE POSITION SUBSYSTEM IMPLEMENTATION

// public class ExamplePositionSubsystem extends PositionSubsystem {

//     private static ExamplePositionSubsystem m_instance = null;

//     public ExamplePositionSubsystem(PositionSubsystemConstants constants) {
//         super(constants);
//     }

//     public static ExamplePositionSubsystem getInstance() {
//         if (m_instance == null) {
//             m_instance = new
// ExamplePositionSubsystem(ExampleConstants.kExamplePositionSubsystemConstants);
//         }

//         return m_instance;
//     }

//     @Override
//     public void subsystemPeriodic() {
//     }

//     @Override
//     public void outputTelemetry() {}

//     public enum ExamplePositionSubsystemState implements PositionSubsystemState {
//         DOWN(0, 0, "Down"),
//         UP(45, 0, "Up"),
//         TRANSITION(0, 0, "Transition"),
//         MANUAL(0, 0, "Manual");

//         private double position;
//         private double velocity;
//         private String name;

//         private ExamplePositionSubsystemState(double position, double velocity, String name) {
//           this.position = position;
//           this.velocity = velocity;
//           this.name = name;
//         }

//         @Override
//         public String getName() {
//             return name;
//         }

//         @Override
//         public double getVelocity() {
//             return velocity;
//         }

//         @Override
//         public void setVelocity(double velocity) {
//             this.velocity = velocity;
//         }

//         @Override
//         public double getPosition() {
//             return position;
//         }

//         @Override
//         public void setPosition(double position) {
//             this.position = position;
//         }
//     }

// }
