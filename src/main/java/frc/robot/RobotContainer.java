// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ManualSubsystem;
import frc.robot.commands.SetSubsystemState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.ArmState;

public class RobotContainer {

  public static final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final Arm m_arm = new Arm(ArmConstants.kArmConstants);
  public static final Elevator m_elevator = new Elevator(ElevatorConstants.kElevatorConstants);
  public static final Wrist m_wrist = new Wrist(WristConstants.kWristConstants);

  public RobotContainer() {

    m_arm.setDefaultCommand(new ManualSubsystem(m_arm));
    m_elevator.setDefaultCommand(new ManualSubsystem(m_elevator));
    m_wrist.setDefaultCommand(new ManualSubsystem(m_wrist));

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().onTrue(new InstantCommand(() -> m_arm.setState(ArmState.OUT)));
    m_driverController.b().onTrue(new InstantCommand(() -> m_arm.setState(ArmState.HOME)));
    m_driverController.x().onTrue(new InstantCommand(() -> m_arm.setState(ArmState.IN)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
