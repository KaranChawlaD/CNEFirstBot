// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final DoubleSolenoid intakePneumatic = new DoubleSolenoid(PneumaticsConstants.PCM,
      PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_FORWARD_PORT, IntakeConstants.INTAKE_REVERSE_PORT);

  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void open() {
    intakePneumatic.set(kForward);
  }

  public void close() {
    intakePneumatic.set(kReverse);
  }

  public void off() {
    intakePneumatic.set(kOff);
  }

  public void toggle() {
    intakePneumatic.toggle();
  }

}
