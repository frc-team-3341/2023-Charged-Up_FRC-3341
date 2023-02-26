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
    public static final int armPort = 2; // ID 2 for testing
    public static final int clawPinchPort = 20;
    public static final int extPort = 4;
    public static final int clawServoPort = 0;
    public static final int wristServoPort = 1;

    public static final int leftStarClawServo = 3;
    public static final int rightStarClawServo = 4;
    public static final int leftFlywheelPort = 30;
    public static final int rightFlywheelPort = 31; 
  }

  public static class PIDConstants {
    public static final double armPID_P = 0.015;
    public static final double armPID_I = 0.00005;
    public static final double armPID_D = 0.0;
    public static final double armPID_K = 0.7; // Used to calculate kP, from the difference in angle
    public static final double armHoldingVoltage = 1.1; // Used to calculate Feedforward
    public static final double armManualHoldingVoltage = 1.9; // Manual holding voltage
  }
  public static class Measurements {
    public static final double threadLength = 0.375; // Inches per rotation of leadscrew
    public static final double threadLengthStarClaw = 0.3; // Inches per rotation of Star Claw leadscrew
    public static final double gearRatio = 1.0/3.0; // 1 rotation of screw = 3 rotations of leadscrew motor
    public static final double lowerAngleBound = 0; // Position of Arm when down

    // When the Arm is extended, this limit is activated
    public static final double bumperAngleBound = 90; // Soft Limit for Arm resting on Bumper

    public static final double upperAngleBound = 160; // Maximum pos of Arm in degrees, when stowed
    public static final double lowerScrewBound = 0.0; // Lower bound for motion of screw (inches)
    public static final double upperScrewBound = 10.0; // Upper bound for motion of screw (inches)
    public static final double fullyExtendedLeadScrewThreshold = 5.0;
    public static final double degreesToTicks = 4096.0/360.0;

    public static final double clawAngleLimit = 80.0; // Limit for servo movement (degrees)

    public static final double starClawPositionLimit = 4.0;

    public static final double wristUpperLimit = 225.0;
    public static final double wristLowerLimit = -225.0; 
  }

  public static class ButtonMap {

    // Trigger button is used for incrementing and decrementing angle of arm

    // Manual override button
    public static final int manualOverride = 2;

    // Buttons for Arm rotation
    public static final int stowPosition = 5;
    public static final int middlePosition = 4;
    public static final int otherArbPosition = 3;
    public static final int groundPosition = 6;

    // Power of extension command for Auto
    // Range: 0.0 to 1.0
    public static final double extensionPower = 0.3; 

    // Potentially temporary buttons
    // Useful for testing autonomous commands
    public static final int fullyExtendedArm = 10;
    public static final int restPositionArm = 9;
    public static final int logButton = 12;

    // Wrist Presets
    public static final int wristLeft = 5;
    public static final int wristRight = 6;
    public static final int wristCenter = 4;
    public static final int clawRest = 3;
    public static final int clawClosed = 2;

    // Powered flywheel control on the left joystick
    public static final int flywheelOn = 7;

    // Seconds until control (angle, etc.) is incremented/decremented
    public static final double controlsDelay = 0.02;

    public static final int wristIncrement = 5; // Wrist increment in degrees for semi-auto control
  }
}
