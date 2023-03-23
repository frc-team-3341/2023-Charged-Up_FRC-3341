// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  DriveTrain dt;
  PIDController pid;
  PIDController yawPID;
  double speed;
  double minSpeed;
  double distance;
  boolean pidDrive;

  public AutoDrive(DriveTrain dt, double distance, boolean pidDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    pid = new PIDController(0.6, 0, 0);
    yawPID = new PIDController(0.02, 0, 0);
    speed = 0.55;
    minSpeed = 0.35;
    this.distance = distance;
    this.pidDrive = pidDrive;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    pid.setSetpoint(dt.getDisplacement() + distance);
    yawPID.setSetpoint(dt.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pidDrive) {
      speed = pid.calculate(dt.getDisplacement());
      if(Math.abs(speed) < minSpeed) speed = minSpeed * Math.signum(speed);
      double turningSpeed = yawPID.calculate(dt.getAngle());
      dt.tankDrive(speed + turningSpeed, speed - turningSpeed);
    }else{
      dt.tankDrive(speed, speed);
    }
    SmartDashboard.putString("Current Command: ", "AutoDrive");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return 0.1 >= Math.abs(distance - dt.getDisplacement()) || Constants.OperatorConstants.angleThreshhold + 3 <= Math.abs(dt.getYAngle());
  }
}
