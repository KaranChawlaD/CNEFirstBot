// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final CANSparkMax elevator = new CANSparkMax(ElevatorConstants.ELEVATOR_RIGHT, MotorType.kBrushless);
  private final RelativeEncoder encoderElevator = elevator.getEncoder();

  public double getEncoderElevatorPosition() {
    return (encoderElevator.getPosition());
  }

  public boolean atTop() {
    return encoderElevator.getPosition() >= ElevatorConstants.ELEVATOR_ENCODER_MAX;
  }

  public ElevatorSubsystem() {
    elevator.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Right Encoder", getEncoderElevatorPosition());
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    elevator.set(speed);
  }

}
