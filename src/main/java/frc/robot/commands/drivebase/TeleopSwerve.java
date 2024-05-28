// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private SwerveDrive m_drivebase;

  private CommandXboxController m_driverController;

  private int m_throttleAxis;
  private int m_strafeAxis;
  private int m_steerAxis;

  private double m_throttle;
  private double m_strafe;
  private double m_steer;

  private boolean m_isOpenLoop;
  private boolean m_isFieldRelative;

  private double m_percentModifier;

  public TeleopSwerve(
      CommandXboxController driverController,
      int throttleAxis,
      int strafeAxis,
      int steerAxis,
      double percentModifier,
      boolean isOpenLoop,
      boolean isFieldRelative) {
    m_drivebase = SwerveDrive.getInstance();

    m_driverController = driverController;

    m_throttleAxis = throttleAxis;
    m_strafeAxis = strafeAxis;
    m_steerAxis = steerAxis;

    m_percentModifier = percentModifier;

    m_isFieldRelative = isFieldRelative;

    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_throttle =
        MathUtil.applyDeadband(
            -m_driverController.getRawAxis(m_throttleAxis), DriveConstants.kSwerveDeadBand);
    m_strafe =
        MathUtil.applyDeadband(
            -m_driverController.getRawAxis(m_strafeAxis), DriveConstants.kSwerveDeadBand);
    m_steer =
        MathUtil.applyDeadband(
            -m_driverController.getRawAxis(m_steerAxis), DriveConstants.kSwerveDeadBand);

    m_throttle *= m_percentModifier;
    m_strafe *= m_percentModifier;
    m_steer *= m_percentModifier;

    m_drivebase.drive(m_throttle, m_strafe, m_steer, m_isOpenLoop, m_isFieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
