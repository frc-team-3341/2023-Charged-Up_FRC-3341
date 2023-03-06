// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RaspberryPi extends SubsystemBase {
  /** Creates a new RaspberryPi. */
  
  private Joystick joystick1 = RobotContainer.getJoy1();
  private Joystick joystick2 = RobotContainer.getJoy2();
  private final PhotonCamera camera;
  private final PhotonCamera camera_1;
  private PhotonTrackedTarget target;

  private boolean hasTarget = false;
  private double yaw; 
  private double pitch; 
  private double area;
  private double skew;
  
  public int pipeline = 0;

  public RaspberryPi() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    camera_1 = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
    camera.setPipelineIndex(0);
    camera_1.setPipelineIndex(1);
    PortForwarder.add(5800,"photonvision.local", 5800);
  }

  public boolean targetExists() {
    return hasTarget;
  }

  public double getYaw() {
    return yaw;
  }

  public double getPitch() {
    return pitch;
  }

  public double getArea() {
    return area;
  }

  public double getSkew() {
    return skew;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult result = camera.getLatestResult();
    hasTarget = result.hasTargets();

    if(hasTarget){

      target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();

    }

    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Area", area);
    SmartDashboard.putNumber("Skew", skew);
    SmartDashboard.putBoolean("Target Acquired", hasTarget);

  }
}