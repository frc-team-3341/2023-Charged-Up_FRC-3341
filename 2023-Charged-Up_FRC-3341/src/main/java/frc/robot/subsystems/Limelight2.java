// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight2 extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-shuzah");
  private Joystick joystick1 = RobotContainer.getJoy1();
  private Joystick joystick2 = RobotContainer.getJoy2();
  private static double txNum2;
  private static double tyNum2;
  private double taNum2;
  private static int tvNum2;
  // Pipeline 0 - reflective tape
  // Pipeline 1 to 7 - april tags
  // Pipeline 8 - cube
  // Pipeline 9 - cone
  public int pipeline = 0;
  private static double CloseReflectiveTapeDistance2;
  private static double FarReflectiveTapeDistance2;
  private static double CloseAprilTagDistance2;
  private static double FarAprilTagDistance2;
  private static double Distance_Test2;

  // This gets the tx, or the horizontal offset
  // from the crosshair in degrees (-27.0 to 27.0)
  NetworkTableEntry tx2 = table.getEntry("tx");

  // This gets the ty, or the vertical offset
  // from the crosshair in degrees (-20.5 to 20.5)
  NetworkTableEntry ty2 = table.getEntry("ty");

  // This gets the ta, or how much in % of the target
  // is visible (0.0-100.0)
  NetworkTableEntry ta2 = table.getEntry("ta");

  // This gets the tv, which sees if the limelight
  // has a valid target (1) or no valid target (0)
  NetworkTableEntry tv2 = table.getEntry("tv");

  public Limelight2() {
    // We have to add these ports so that we can connect to
    // the limelight with our code through the robot's wifi
    // PortForwarder.add(5800, "http://10.33.41.11:5801/", 5800);
    PortForwarder.add(5801, "http://limelight-shuzah.local:5801", 5801);
    PortForwarder.add(5802, "http://limelight-shuzah.local:5801", 5802);
    PortForwarder.add(5803, "http://limelight-shuzah.local:5801", 5803);
    PortForwarder.add(5804, "http://limelight-shuzah.local:5801", 5804);
    PortForwarder.add(5805, "http://limelight-shuzah.local:5801", 5805);
    PortForwarder.add(5800, "http://limelight-shuzah.local:5801", 5800);

    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("Stream").setNumber(2);
  }
  
  public void changepipeline(int pipeline2){ table.getEntry("pipeline2").setNumber(pipeline2); }
  public static double get_tx2(){ return txNum2; }
  public static double get_ty2(){ return tyNum2; }
  public double get_ta2(){ return taNum2; }
  public static int get_tv2(){ return tvNum2; }
  public static double getCloseReflectiveTapeDistance2(){ return CloseReflectiveTapeDistance2; }
  public static double getFarReflectiveTapeDistance2(){ return FarReflectiveTapeDistance2; }
  public static double getCloseAprilTagDistance2(){ return CloseAprilTagDistance2; }
  public static double getFarAprilTagDistance2(){ return FarAprilTagDistance2; }
  public static double getDistance_Test2(){ return Distance_Test2; }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
// 1st (closest) reflective tape pole
    double pole_1_targetOffsetAngle_Vertical2 = ty2.getDouble(0.0);
    // How many degrees back is your limelight rotated from perfectly vertical?
    double pole_1_limelightMountAngleDegrees2 = 3.15;
    // Distance from the center of the Limelight lens to the floor
    double pole_1_limelightLensHeightInches2 = 7.165354;
    // Distance from the target to the floor
    double pole_1_goalHeightInches2 = 34.0;
    double pole_1_angleToGoalDegrees2 = pole_1_limelightMountAngleDegrees2 + pole_1_targetOffsetAngle_Vertical2;
    // Converts degrees to radians
    double pole_1_angleToGoalRadians2 = pole_1_angleToGoalDegrees2 * (Math.PI / 180.0);
    // Calculates distance
    CloseReflectiveTapeDistance2 = (pole_1_goalHeightInches2 - pole_1_limelightLensHeightInches2)/Math.tan(pole_1_angleToGoalRadians2);
    // This outputs the distance from the limelight to the target
    SmartDashboard.putNumber("CloseReflectiveTapeDistance2 (inches)", CloseReflectiveTapeDistance2);

// 2nd (farthest) reflective tape pole
    double pole_2_targetOffsetAngle_Vertical2 = ty2.getDouble(0.0);
    double pole_2_limelightMountAngleDegrees2 = 3.15; 
    double pole_2_limelightLensHeightInches2 = 7.165354;
    double pole_2_goalHeightInches2 = 46;
    double pole_2_angleToGoalDegrees2 = pole_2_limelightMountAngleDegrees2 + pole_2_targetOffsetAngle_Vertical2;
    double pole_2_angleToGoalRadians2 = pole_2_angleToGoalDegrees2 * (Math.PI / 180.0);
    FarReflectiveTapeDistance2 = (pole_2_goalHeightInches2 - pole_2_limelightLensHeightInches2)/Math.tan(pole_2_angleToGoalRadians2);
    SmartDashboard.putNumber("FarReflectiveTapeDistance2 (inches)", FarReflectiveTapeDistance2);

// 3rd (closest) april tag shelf
    double pole_3_targetOffsetAngle_Vertical2 = ty2.getDouble(0.0);
    double pole_3_limelightMountAngleDegrees2 = 3.15;
    double pole_3_limelightLensHeightInches2 = 7.165354;
    double pole_3_goalHeightInches2 = 23.5;
    double pole_3_angleToGoalDegrees2 = pole_3_limelightMountAngleDegrees2 + pole_3_targetOffsetAngle_Vertical2;
    double pole_3_angleToGoalRadians2 = pole_3_angleToGoalDegrees2 * (Math.PI / 180.0);
    CloseAprilTagDistance2 = (pole_3_goalHeightInches2 - pole_3_limelightLensHeightInches2)/Math.tan(pole_3_angleToGoalRadians2);
    SmartDashboard.putNumber("CloseAprilTagDistance2 (inches)", CloseAprilTagDistance2);

// 4th (farthest) april tag shelf
    double pole_4_targetOffsetAngle_Vertical2 = ty2.getDouble(0.0);
    double pole_4_limelightMountAngleDegrees2 = 3.15; 
    double pole_4_limelightLensHeightInches2 = 7.165354;
    double pole_4_goalHeightInches2 = 35.5;
    double pole_4_angleToGoalDegrees2 = pole_4_limelightMountAngleDegrees2 + pole_4_targetOffsetAngle_Vertical2;
    double pole_4_angleToGoalRadians2 = pole_4_angleToGoalDegrees2 * (Math.PI / 180.0);
    FarAprilTagDistance2 = (pole_4_goalHeightInches2 - pole_4_limelightLensHeightInches2)/Math.tan(pole_4_angleToGoalRadians2);
    SmartDashboard.putNumber("FarAprilTagDistance2 (inches)", FarAprilTagDistance2);

// test
    double test_targetOffsetAngle_Vertical2 = ty2.getDouble(0.0);
    double test_limelightMountAngleDegrees2 = 3.15;
    double test_limelightLensHeightInches2 = 7.165354;
    double test_goalHeightInches2 = 15;
    double test_angleToGoalDegrees2 = test_limelightMountAngleDegrees2 + test_targetOffsetAngle_Vertical2;
    double test_angleToGoalRadians2 = test_angleToGoalDegrees2 * (Math.PI / 180.0);
    Distance_Test2 = (test_goalHeightInches2 - test_limelightLensHeightInches2)/Math.tan(test_angleToGoalRadians2);
    SmartDashboard.putNumber("Distance_Test2 (inches)", Distance_Test2);

    tx2 = table.getEntry("tx2");
    ty2 = table.getEntry("ty2");
    ta2 = table.getEntry("ta2");
    tv2 = table.getEntry("tv2");

    // tvNum can be either 1 or 0, so we instantiate by adding (int) in front
    // We will be assigning tvNum to the int (1 or 0) that limelight returns
    tvNum2 = (int) tv2.getDouble(0.0);

    // We will be assigning tyNum to the double (-27.0 to 27.0) that limelight returns
    txNum2 = tx2.getDouble(0.0);

    // We will be assigning tyNum to the double (-20.5 to 20.5) that limelight returns
    tyNum2 = ty2.getDouble(0.0);

    // We will be assigning taNum to the double (0.0-100.0) that limelight returns
    taNum2 = ta2.getDouble(0.0);

    // This will output the x (horizontal offset) from the target in SmartDashboard
    SmartDashboard.putNumber("LimelightX2", txNum2);

    // This will output the y (vertical offset) from the target in SmartDashboard
    SmartDashboard.putNumber("LimelightY2", tyNum2);

    // This will output the area of the target in SmartDashboard
    SmartDashboard.putNumber("LimelightArea2", taNum2);

    // This will output the value of the target in SmartDashboard (0 or 1)
    SmartDashboard.putNumber("LimelightV2", tvNum2);

    // SmartDashboard.putNumber("PipelineNumber", pipeline);
    // Actual pipeline number not representative
    SmartDashboard.putNumber("PipelineName2", table.getEntry("pipeline2").getDouble(0));// Actual piepline

    NetworkTableInstance.getDefault().getTable("limelight-shuzah").getEntry("tx").getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight-shuzah").getEntry("ty").getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight-shuzah").getEntry("tv").getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight-shuzah").getEntry("ta").getDouble(0);
    
  }
}