package frc.lib.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatedSubsystemUtils {
  public static record PositionSubsystemState(double position, String name) {}

  public abstract static class PositionSubsystem extends SubsystemBase {

    protected String m_currentState;
    protected String m_desiredState;

    protected double m_desiredPosition;

    private double m_setpointTolerance;

    protected PositionSubsystem(double setpointTolerance) {
      m_setpointTolerance = setpointTolerance;
    }

    public boolean getIsAtSetpoint() {
      return Math.abs(getPosition() - m_desiredPosition) < m_setpointTolerance;
    }

    public abstract void runToPosition(double position);

    public abstract void runProfileToPosition(double position);

    public abstract double getPosition();

    public void setCurrentState(String currentState) {
      m_currentState = currentState;
    }

    public void setDesiredState(String desiredState) {
      m_desiredState = desiredState;
    }

    public Command runProfileToPositionCommand(PositionSubsystemState state) {
      return new FunctionalCommand(
          () -> {
            runProfileToPosition(state.position);
            setCurrentState("Transition");
            setDesiredState(state.name);
            m_desiredPosition = state.position;
          },
          () -> {},
          (interrupted) -> {
            if (!interrupted) {
              setCurrentState(state.name);
            }
          },
          this::getIsAtSetpoint,
          this);
    }
  }

  public static record VelocitySubsystemState(String name, double... velocity) {
    public VelocitySubsystemState {
      if (velocity == null) {
        throw new IllegalArgumentException("velocity cannot be null");
      }
    }
  }

  public abstract static class VelocitySubsystem extends SubsystemBase {
    protected String m_currentState;
    protected String m_desiredState;

    public abstract boolean getIsAtSetpoint();

    public abstract double[] getVelocity();

    public abstract void runToVelocity(double... velocity);

    public void setCurrentState(String currentState) {
      m_currentState = currentState;
    }

    public void setDesiredState(String desiredState) {
      m_desiredState = desiredState;
    }

    public Command runToVelocityCommand(VelocitySubsystemState state) {
      return new FunctionalCommand(
              () -> runToVelocity(state.velocity),
              () -> {},
              (interrupted) -> {},
              this::getIsAtSetpoint,
              this)
          .beforeStarting(
              () -> {
                setCurrentState("TRANSITION");
                setDesiredState(state.name);
              })
          .finallyDo(() -> setCurrentState(state.name));
    }
  }

  public static record VoltageSubsystemState(double voltage, String name) {}

  public abstract static class VoltageSubsystem extends SubsystemBase {
    protected String m_currentState;

    public abstract double getVoltage();

    public abstract void setVoltage(double voltage);

    public void setCurrentState(String currentState) {
      m_currentState = currentState;
    }

    public String getCurrentState() {
      return m_currentState;
    }

    public Command setVoltageCommand(VoltageSubsystemState state) {
      Command output =
          new InstantCommand(() -> setVoltage(state.voltage))
              .finallyDo(() -> setCurrentState(state.name));

      output.addRequirements(this);

      return output;
    }
  }
}
