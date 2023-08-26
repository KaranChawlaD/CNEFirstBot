// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Turn90 extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final boolean right;

  /** Creates a new Turn90. */
  public Turn90(DriveSubsystem driveSubsystem, boolean right) {
    this.driveSubsystem = driveSubsystem;
    this.right = right;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (right) {
      driveSubsystem.setMotor(0, -driveSubsystem.getTurnControllerSpeedRight());
    } else {
      driveSubsystem.setMotor(0, -driveSubsystem.getTurnControllerSpeedLeft());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.atSetpoint();
  }
}
