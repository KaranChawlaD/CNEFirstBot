// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.RotationConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class RotationSubsystem extends SubsystemBase {
  /** Creates a new RotationSubsystem. */
  private final DoubleSolenoid rotationPneumatic = new DoubleSolenoid(PneumaticsConstants.PCM,
      PneumaticsModuleType.CTREPCM, RotationConstants.ROTATION_FORWARD_PORT, RotationConstants.ROTATION_REVERSE_PORT);

  public RotationSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    rotationPneumatic.set(kForward);
  }

  public void retract() {
    rotationPneumatic.set(kReverse);
  }

  public void off() {
    rotationPneumatic.set(kOff);
  }

  public void toggle() {
    rotationPneumatic.toggle();
  }

}
