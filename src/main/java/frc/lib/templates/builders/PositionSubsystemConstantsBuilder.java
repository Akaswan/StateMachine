package frc.lib.templates.builders;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.templates.subsystems.PositionSubsystem.PositionSubsystemState;
import frc.lib.templates.subsystems.SubsystemConstants.ManualControlMode;
import frc.lib.templates.subsystems.SubsystemConstants.MotorConstants;
import frc.lib.templates.subsystems.SubsystemConstants.PositionSubsystemConstants;

public class PositionSubsystemConstantsBuilder {
  private final PositionSubsystemConstants constants;

  public PositionSubsystemConstantsBuilder() {
    constants = new PositionSubsystemConstants();
  }

  public PositionSubsystemConstantsBuilder withSuperstructureName(String name) {
    constants.kSuperstructureName = name;
    return this;
  }

  public PositionSubsystemConstantsBuilder withSubsystemName(String name) {
    constants.kSubsystemName = name;
    return this;
  }

  public PositionSubsystemConstantsBuilder withLeaderConstants(MotorConstants motorConstants) {
    constants.kLeaderConstants = motorConstants;
    return this;
  }

  public PositionSubsystemConstantsBuilder withFollowerConstants(MotorConstants... motorConstants) {
    constants.kFollowerConstants = motorConstants;
    return this;
  }

  public PositionSubsystemConstantsBuilder withInitialState(PositionSubsystemState state) {
    constants.kInitialState = state;
    return this;
  }

  public PositionSubsystemConstantsBuilder withSetpointTolerance(double tolerance) {
    constants.kSetpointTolerance = tolerance;
    return this;
  }

  public PositionSubsystemConstantsBuilder withProfileConstraints(
      TrapezoidProfile.Constraints constraints) {
    constants.kProfileConstraints = constraints;
    return this;
  }

  public PositionSubsystemConstantsBuilder withMinPosition(double min) {
    constants.kMinPosition = min;
    return this;
  }

  public PositionSubsystemConstantsBuilder withMaxPosition(double max) {
    constants.kMaxPosition = max;
    return this;
  }

  public PositionSubsystemConstantsBuilder withManualControlMode(ManualControlMode mode) {
    constants.kManualControlMode = mode;
    return this;
  }

  public PositionSubsystemConstantsBuilder withmanualMultiplier(double multiplier) {
    constants.kManualMultiplier = multiplier;
    return this;
  }

  public PositionSubsystemConstantsBuilder withManualDeadband(double deadband) {
    constants.kManualDeadBand = deadband;
    return this;
  }

  public PositionSubsystemConstants build() {
    return constants;
  }
}
