// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveEncoderCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final double distance;
  private final double speed;

  /** Creates a new DriveEncoderCommand. */
  public DriveEncoderCommand(DriveSubsystem driveSubsystem, double speed, double distance) {
    this.driveSubsystem = driveSubsystem;
    this.speed = speed;
    this.distance = distance;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setMotor(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveSubsystem.getEncoderDrivePosition() < distance;
  }
}
