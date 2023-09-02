// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RotationConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.BalanceDrive;
import frc.robot.commands.ConeAutomation;
import frc.robot.commands.DoubleSubstationCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PCMSubsystem;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.Stopwatch;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.EnableCompressorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.Turn90;
import frc.robot.commands.EngageAuto;
import frc.robot.commands.MobilityAuto;
import frc.robot.commands.RetractAutomation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final RotationSubsystem rotationSubsystem = new RotationSubsystem();
        private final PCMSubsystem pcmSubsystem = new PCMSubsystem();
        private final Stopwatch timer = new Stopwatch();

        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final Joystick driverController = new Joystick(ControllerConstants.DRIVER_CONTROLLER_PORT);

        private final Joystick operatorController = new Joystick(ControllerConstants.OPERATOR_CONTROLLER_PORT);

        // A chooser for autonomous commands
        SendableChooser<Command> chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();

                driveSubsystem.setDefaultCommand(
                                new ArcadeDriveCommand(driveSubsystem,
                                                () -> -driverController.getRawAxis(DriveConstants.DRIVE_AXIS),
                                                (() -> -driverController.getRawAxis(DriveConstants.TURN_AXIS)
                                                                * DriveConstants.TURN_PROPORTION)));

                pcmSubsystem.setDefaultCommand(new EnableCompressorCommand(pcmSubsystem, true));

                // Autonomous Chooser Options
                chooser.setDefaultOption("Engage Auto",
                                new EngageAuto(driveSubsystem, elevatorSubsystem, rotationSubsystem,
                                                intakeSubsystem, timer));

                chooser.addOption("Mobility Auto",
                                new MobilityAuto(driveSubsystem, elevatorSubsystem, rotationSubsystem,
                                                intakeSubsystem, timer));

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(chooser);
        }

        private void configureBindings() {

                // Elevator
                new JoystickButton(operatorController, ElevatorConstants.ELEVATOR_UP_BUTTON)
                                .whileTrue(new ElevatorCommand(elevatorSubsystem, ElevatorConstants.ELEVATOR_UP_SPEED));
                new JoystickButton(operatorController, ElevatorConstants.ELEVATOR_DOWN_BUTTON)
                                .whileTrue(new ElevatorCommand(elevatorSubsystem,
                                                ElevatorConstants.ELEVATOR_DOWN_SPEED));

                // Rotation
                new JoystickButton(operatorController, RotationConstants.ROTATION_RETRACT_BUTTON)
                                .onTrue(new RotationCommand(rotationSubsystem, false));
                new JoystickButton(operatorController, RotationConstants.ROTATION_EXTEND_BUTTON)
                                .onTrue(new RotationCommand(rotationSubsystem, true));

                // Compressor
                new JoystickButton(operatorController, RotationConstants.COMPRESSOR_ON_BUTTON)
                                .onTrue(new EnableCompressorCommand(pcmSubsystem, true));
                new JoystickButton(operatorController, RotationConstants.COMPRESSOR_OFF_BUTTON)
                                .onTrue(new EnableCompressorCommand(pcmSubsystem, false));

                // Intake
                new JoystickButton(operatorController, IntakeConstants.INTAKE_CLOSE_BUTTON)
                                .onTrue(new IntakeCommand(intakeSubsystem, false));
                new JoystickButton(operatorController, IntakeConstants.INTAKE_OPEN_BUTTON)
                                .onTrue(new IntakeCommand(intakeSubsystem, true));

                // Automated Commands (driver then operator)
                new JoystickButton(driverController, AutoConstants.DOUBLE_SUBSTATION_BUTTON)
                                .onTrue(new DoubleSubstationCommand(rotationSubsystem, intakeSubsystem,
                                                elevatorSubsystem));
                new JoystickButton(driverController, DriveConstants.TURN_RIGHT_90_BUTTON)
                                .onTrue(new Turn90(driveSubsystem, true).withTimeout(1));
                new JoystickButton(driverController, DriveConstants.TURN_LEFT_90_BUTTON)
                                .onTrue(new Turn90(driveSubsystem, false).withTimeout(1));

                new JoystickButton(operatorController, AutoConstants.CONE_AUTOMATION_BUTTON)
                                .onTrue(new ConeAutomation(rotationSubsystem, elevatorSubsystem));
                new JoystickButton(operatorController, AutoConstants.RETRACT_AUTOMATION_BUTTON)
                                .onTrue(new RetractAutomation(rotationSubsystem, elevatorSubsystem, intakeSubsystem));
                
                // Testing Balance
                new JoystickButton(driverController, 1).onTrue(new BalanceDrive(driveSubsystem));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return chooser.getSelected();
        }
}
