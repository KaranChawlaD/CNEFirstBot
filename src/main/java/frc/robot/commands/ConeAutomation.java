// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConeAutomation extends ParallelCommandGroup {
  /** Creates a new DoubleSubstationCommand. */
  public ConeAutomation(RotationSubsystem rotationSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RotationCommand(rotationSubsystem, true),
        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.ELEVATOR_UP_SPEED).withTimeout(1.5));
  }
}
