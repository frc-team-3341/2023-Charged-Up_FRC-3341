// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-drsolom");
  private Joystick joystick1 = RobotContainer.getJoy1();
  private Joystick joystick2 = RobotContainer.getJoy2();

  private static double txNum;
  private static double tyNum;
  private double taNum;
  private static int tvNum;
  // Pipeline 0 - reflective tape
  // Pipeline 1 to 7 - april tags
  // Pipeline 8 - cube
  // Pipeline 9 - cone
  public int pipeline = 0;

  // This gets the tx, or the horizontal offset
  // from the crosshair in degrees (-27.0 to 27.0)
  NetworkTableEntry tx = table.getEntry("tx");

  // This gets the ty, or the vertical offset
  // from the crosshair in degrees (-20.5 to 20.5)
  NetworkTableEntry ty = table.getEntry("ty");

  // This gets the ta, or how much in % of the target
  // is visible (0.0-100.0)
  NetworkTableEntry ta = table.getEntry("ta");

  // This gets the tv, which sees if the limelight
  // has a valid target (1) or no valid target (0)
  NetworkTableEntry tv = table.getEntry("tv");

  public Limelight() {
    // We have to add these ports so that we can connect to
    // the limelight with our code through the robot's wifi
    // PortForwarder.add(5800, "http://10.33.41.11:5801/", 5800);
    PortForwarder.add(5801, "http://limelight-drsolom.local:5801", 5801);
    PortForwarder.add(5802, "http://limelight-drsolom.local:5801", 5802);
    PortForwarder.add(5803, "http://limelight-drsolom.local:5801", 5803);
    PortForwarder.add(5804, "http://limelight-drsolom.local:5801", 5804);
    PortForwarder.add(5805, "http://limelight-drsolom.local:5801", 5805);
    PortForwarder.add(5800, "http://limelight-drsolom.local:5801", 5800);
  }
  
  
  public void changepipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public static double get_tx() {
    return txNum;
  }

  public static double get_ty() {
    return tyNum;
  }

  public double get_ta() {
    return taNum;
  }

  public static int get_tv() {
    return tvNum;
  }

  public void cycleOddPipe() {
    for (int i = 0; i < 4; i++) {
      changepipeline(2 * i + 1);
      if (tvNum == 1)
        return;
    }
  }

  public void cycleEvenPipe() {
    for (int i = 0; i < 4; i++) {
      changepipeline(2 * i + 2);
      if (tvNum == 1)
        return;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /*if (joystick1.getRawButtonPressed(3)) {
      pipeline = 0; // reflective tape
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(4)) {
      pipeline = 9; // cone
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(5)) {
      pipeline = 8; // square
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(12)) {
      pipeline = 1; // april tag 1
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(11)) {
      pipeline = 2; // april tag 2
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(10)) {
      pipeline = 3; // april tag 3
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(9)) {
      pipeline = 4; // april tag 4
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(8)) {
      pipeline = 5; // april tag 5
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(7)) {
      pipeline = 6; // april tag 6
      changepipeline(pipeline);
    } else if (joystick1.getRawButtonPressed(6)) {
      pipeline = 7; // april tag 7
      changepipeline(pipeline);
    }*/

  /*  if(joystick1.getRawButtonPressed(3)){
      cycleOddPipe();
    }
    if(joystick1.getRawButtonPressed(4)){
      cycleEvenPipe();
    } */

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");

    // tvNum can be either 1 or 0, so we instantiate by adding (int) in front
    // We will be assigning tvNum to the int (1 or 0) that limelight returns
    tvNum = (int) tv.getDouble(0.0);

    // We will be assigning tyNum to the double (-27.0 to 27.0) that limelight
    // returns
    txNum = tx.getDouble(0.0);

    // We will be assigning tyNum to the double (-20.5 to 20.5) that limelight
    // returns
    tyNum = ty.getDouble(0.0);

    // We will be assigning taNum to the double (0.0-100.0) that limelight returns
    taNum = ta.getDouble(0.0);
    // This will output the x (horizontal offset) from the target in SmartDashboard
    SmartDashboard.putNumber("LimelightX", txNum);

    // This will output the y (vertical offset) from the target in SmartDashboard
    SmartDashboard.putNumber("LimelightY", tyNum);

    // This will output the area of the target in SmartDashboard
    SmartDashboard.putNumber("LimelightArea", taNum);

    // This will output the value of the target in SmartDashboard (0 or 1)
    SmartDashboard.putNumber("LimelightV", tvNum);

    // SmartDashboard.putNumber("PipelineNumber", pipeline);
    // Actual pipeline number not representative
    SmartDashboard.putNumber("PipelineName", table.getEntry("pipeline").getDouble(0));// Actual piepline




  }
}