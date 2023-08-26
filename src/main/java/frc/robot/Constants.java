// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class PneumaticsConstants {
    public static final int PCM = 1;
  }

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class DriveConstants {
    public static final int DRIVE_FRONT_LEFT = 2;
    public static final int DRIVE_FRONT_RIGHT = 4;
    public static final int DRIVE_BACK_LEFT = 3;
    public static final int DRIVE_BACK_RIGHT = 5;
    public static final int LEFT_ENCODER_A = 1;
    public static final int LEFT_ENCODER_B = 2;
    public static final int DRIVE_AXIS = 1;
    public static final int TURN_AXIS = 4;
    public static final double TURN_PROPORTION = 0.7;
    public static final double DRIVE_SLOW = 0.5;
    public static final double TURN_SLOW = 0.8;
    public static final double TURN_KP = 0.017;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
    public static final int TURN_CONTROLLER_POSITION_TOLERANCE = 1;
    public static final int TURN_CONTROLLER_VELOCITY_TOLERANCE = 10;
    public static final int TURN_RIGHT_90_BUTTON = 2;
    public static final int TURN_LEFT_90_BUTTON = 3;

  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_RIGHT = 6;
    public static final int ELEVATOR_UP_BUTTON = 4;
    public static final int ELEVATOR_DOWN_BUTTON = 2;
    public static final double ELEVATOR_UP_SPEED = -0.2;
    public static final double ELEVATOR_DOWN_SPEED = 0.1;
    public static final double ELEVATOR_STALL_SPEED = -0.03;
    public static final double ELEVATOR_ENCODER_MAX = 0; // Encoder Value When Elevator is at Top
  }

  public static class IntakeConstants {
    public static final int INTAKE = 3;
    public static final int INTAKE_OPEN_BUTTON = 8;
    public static final int INTAKE_CLOSE_BUTTON = 6;
    public static final double CUBE_IN_SPEED = 0.4;
    public static final double CUBE_IN_STALL = 0.2;
    public static final double CONE_IN_SPEED = -0.5;
    public static final double CONE_IN_STALL = -0.2;
    public static final int INTAKE_FORWARD_PORT = 2; // To Wire
    public static final int INTAKE_REVERSE_PORT = 3; // To Wire
  }

  public static class RotationConstants {
    public static final int ROTATION = 13;
    public static final int COMPRESSOR_ON_BUTTON = 10;
    public static final int COMPRESSOR_OFF_BUTTON = 9;
    public static final int ROTATION_EXTEND_BUTTON = 5;
    public static final int ROTATION_RETRACT_BUTTON = 7;
    public static final double ROTATION_UP_SPEED = 0.95;
    public static final double ROTATION_DOWN_SPEED = -0.55;
    public static final int ROTATION_FORWARD_PORT = 0; // To Wire
    public static final int ROTATION_REVERSE_PORT = 1; // To Wire
  }

  public static class AutoConstants {
    // Double Substation
    public static final int DOUBLE_SUBSTATION_BUTTON = 5;
    public static final int CONE_AUTOMATION_BUTTON = 3;
    public static final int RETRACT_AUTOMATION_BUTTON = 1;
  }
}
