// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Drivetrain {
    public static final int LEFT_MOTOR_0_ID = -1; //TOBO: Get Device ID
    public static final int LEFT_MOTOR_1_ID = -1; //TOBO: Get Device ID
    public static final int RIGHT_MOTOR_0_ID = -1; //TOBO: Get Device ID
    public static final int RIGHT_MOTOR_1_ID = -1; //TOBO: Get Device ID

    public static final double TRACK_WIDTH = -1.0; // todo get track width

    public static final double LEFT_KP = -1.0; // todo tune left kp
    public static final double RIGHT_KP = -1.0; // todo tune right kp

    public static final double wheelDiameter = -1.0; // todo: get wheel diameter
    public static final double gearratio = -1.0/0; // todo get gear ratio

  }
  //balls
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

}
