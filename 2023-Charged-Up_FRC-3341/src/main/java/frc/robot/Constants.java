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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int armPort = 2;
    public static final int extPort = 4;
  }

  public static class PIDConstants {
    public static final double armPID_P = 0.015;
    public static final double armPID_I = 0.00005;
    public static final double armPID_D = 0.0;
  }
  public static class Measurements {
    public static final double threadLength = 0.138; //inches per rotation
    public static final double gearRatio = 1/3; //1 rotation of screw = 3 rotations of motor
  }
}
