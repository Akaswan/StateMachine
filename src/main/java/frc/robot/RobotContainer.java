// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ManualSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.manager.ServoMotorSubsystem;
import frc.robot.subsystems.manager.SuperstructureStateManager;
import frc.robot.subsystems.manager.SuperstructureStateManager.SuperstructureState;

public class RobotContainer {

  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  public static final Arm m_arm = new Arm(ArmConstants.kArmConstants);
  public static final Elevator m_elevator = new Elevator(ElevatorConstants.kElevatorConstants);
  public static final Wrist m_wrist = new Wrist(WristConstants.kWristConstants);

  public static final SuperstructureStateManager m_manager = new SuperstructureStateManager(SuperstructureState.HOME);

  public RobotContainer() {

    m_arm.setDefaultCommand(new ManualSubsystem(m_arm));
    m_elevator.setDefaultCommand(new ManualSubsystem(m_elevator));
    m_wrist.setDefaultCommand(new ManualSubsystem(m_wrist));

    configureBindings();
  }

  private void configureBindings() {
    // m_driverController.y().onTrue(m_manager.setSuperstructureState(new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist}, SuperstructureState.PLACE));
    // m_driverController.x().onTrue(m_manager.setSuperstructureState(new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist}, SuperstructureState.HOME));
    // m_driverController.b().onTrue(m_manager.setSuperstructureState(new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist}, SuperstructureState.PICKUP));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
    // SmartDashboard.putString("Current State", m_manager.getCurrentState().getName());
    // SmartDashboard.putString("Last State", m_manager.getLastHeldState().getName());
    // SmartDashboard.putString("Desired State", m_manager.getDesiredState().getName());
    // SmartDashboard.putNumber("Arm Transition Position", m_manager.getCurrentState().getArmState().getPosition());
    // SmartDashboard.putString("Arm Subsystem transtiion name", m_manager.getCurrentState().getArmState().getName());
    // SmartDashboard.putNumber("Elevator Transition Position", m_manager.getCurrentState().getElevatorState().getPosition());
    // SmartDashboard.putNumber("Wrist Transition Position", m_manager.getCurrentState().getWristState().getPosition());
    // SmartDashboard.putNumber("Poop test", ArmState.TRANSITION.getPosition());
    // SmartDashboard.putString("Arm current state", m_arm.getCurrentState().getName());
    // SmartDashboard.putNumber("Arm current position", m_arm.getCurrentState().getPosition());

    if (m_operatorController.getYButtonPressed()) {
      m_manager.setSuperstructureState(new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist}, SuperstructureState.PLACE).schedule();
    }
    if (m_operatorController.getXButtonPressed()) {
      m_manager.setSuperstructureState(new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist}, SuperstructureState.HOME).schedule();
    }
    if (m_operatorController.getBButtonPressed()) {
      m_manager.setSuperstructureState(new ServoMotorSubsystem[] {m_arm, m_elevator, m_wrist}, SuperstructureState.PICKUP).schedule();
    }

  }
}
