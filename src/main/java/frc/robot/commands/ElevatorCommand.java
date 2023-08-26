// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double speed;

  /** Creates a new ElevatorCommand. */

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.speed = speed;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotor(ElevatorConstants.ELEVATOR_STALL_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
