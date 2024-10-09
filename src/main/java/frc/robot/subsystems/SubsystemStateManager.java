package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeWheels;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.intake.TransferWheels;
import org.littletonrobotics.junction.Logger;

public class SubsystemStateManager extends SubsystemBase {
  private static RobotState m_currentState = RobotState.STOWED;
  private static RobotState m_desiredState = RobotState.STOWED;

  public static Command intake(
      IntakeWrist intakeWrist, IntakeWheels intakeWheels, TransferWheels transferWheels) {
    return wrapWithState(
        intakeWrist
            .intakeDown()
            .alongWith(intakeWheels.wheelsIntaking(), transferWheels.wheelsToLauncher()),
        RobotState.INTAKING);
  }

  public static Command stow(
      IntakeWrist intakeWrist, IntakeWheels intakeWheels, TransferWheels transferWheels) {
    return wrapWithState(
        intakeWrist.intakeStowed().alongWith(intakeWheels.wheelsOff(), transferWheels.wheelsOff()),
        RobotState.STOWED);
  }

  private static Command wrapWithState(Command command, RobotState state) {
    return command
        .beforeStarting(
            () -> {
              m_desiredState = state;
              m_currentState = RobotState.TRANSITION;
            })
        .finallyDo(() -> m_currentState = state);
  }

  public static RobotState getCurrentState() {
    return m_currentState;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Current Robot State", m_currentState.name());
    Logger.recordOutput("Desired Robot State", m_desiredState.name());
  }

  public enum RobotState {
    STOWED,
    TRANSITION,
    INTAKING
  }
}
