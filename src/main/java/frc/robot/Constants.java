// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveModuleConstants {

    // Front left
    public static final int VELOCITY_MOTOR_ID_FL;
    public static final int ANGULAR_MOTOR_ID_FL;
    public static final int ANGULAR_MOTOR_ENCODER_ID_FL;
    public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FL;

    // Front right
    public static final int VELOCITY_MOTOR_ID_FR;
    public static final int ANGULAR_MOTOR_ID_FR;
    public static final int ANGULAR_MOTOR_ENCODER_ID_FR;
    public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FR;

    // Back left
    public static final int VELOCITY_MOTOR_ID_BL;
    public static final int ANGULAR_MOTOR_ID_BL;
    public static final int ANGULAR_MOTOR_ENCODER_ID_BL;
    public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BL;

    // Back right
    public static final int VELOCITY_MOTOR_ID_BR;
    public static final int ANGULAR_MOTOR_ID_BR;
    public static final int ANGULAR_MOTOR_ENCODER_ID_BR;
    public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BR;

    public static final double MODULE_LOCATION_X = 54.0 / 100.0;
    public static final double MODULE_LOCATION_Y = 54.0 / 100.0;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

    static {
      // Front Left
      VELOCITY_MOTOR_ID_FL = 2;
      ANGULAR_MOTOR_ID_FL = 3;
      ANGULAR_MOTOR_ENCODER_ID_FL = 3;
      ANGULAR_MOTOR_ENCODER_OFFSET_FL = 0.881591796875 - 0.25;

      // Front right
      VELOCITY_MOTOR_ID_FR = 14; // Was 16
      ANGULAR_MOTOR_ID_FR = 17;
      ANGULAR_MOTOR_ENCODER_ID_FR = 4;
      ANGULAR_MOTOR_ENCODER_OFFSET_FR = -0.77758789062;

      // Back left
      VELOCITY_MOTOR_ID_BL = 8;
      ANGULAR_MOTOR_ID_BL = 9;
      ANGULAR_MOTOR_ENCODER_ID_BL = 2;
      ANGULAR_MOTOR_ENCODER_OFFSET_BL = -0.641357421875;

      // Back right
      VELOCITY_MOTOR_ID_BR = 10;
      ANGULAR_MOTOR_ID_BR = 11;
      ANGULAR_MOTOR_ENCODER_ID_BR = 1;
      ANGULAR_MOTOR_ENCODER_OFFSET_BR = -0.046142578125 + 0.5;
    }
  }
}
