// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MobilityAuto extends SequentialCommandGroup {
    /** Creates a new MobilityAuto. */
    public MobilityAuto(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem,
            RotationSubsystem rotationSubsystem, IntakeSubsystem intakeSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ConeAutomation(rotationSubsystem, elevatorSubsystem),
                new ArcadeDriveCommand(driveSubsystem, () -> 0.4, () -> 0.0).withTimeout(0.75),
                new IntakeCommand(intakeSubsystem, true),
                new ParallelCommandGroup(
                        new ArcadeDriveCommand(driveSubsystem, () -> -0.6, () -> 0.0).withTimeout(3.5),
                        new ElevatorCommand(elevatorSubsystem, ElevatorConstants.ELEVATOR_DOWN_SPEED).withTimeout(1.5)),
                new Turn90(driveSubsystem, true),
                new Turn90(driveSubsystem, true));
    }
}
