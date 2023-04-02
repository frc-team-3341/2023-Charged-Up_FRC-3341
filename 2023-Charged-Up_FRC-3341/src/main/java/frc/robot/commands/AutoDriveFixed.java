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

public class AutoDriveFixed extends CommandBase {
  /** Creates a new AutoDrive. */
  DriveTrain dt;
  PIDController pid;
  PIDController yawPID;
  double speed;
  double minSpeed;
  double distance;
  boolean pidDrive;
  boolean finishCommand;
  double error;

  /**
   * Auto Drives the chassis to a set position (meters)
   * @param dt - DriveTrain
   * @param distance - Distance Travelled
   * @param speed - Power at which the robot drives
   * @param pidDrive - Whether or not to use PID Controller
   * @param finishCommand - Whether or not to not finish the command
   * @apiNote For finishCommand: true - Command does not finish
   * @apiNote For finishCommand: false - Command does finish
   */
  public AutoDriveFixed(DriveTrain dt, double distance, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    //pid = new PIDController(0.6, 0, 0);
    //yawPID = new PIDController(0.02, 0, 0);
    this.speed = speed;
    minSpeed = 0.35;
    this.distance = distance;
    //this.pidDrive = pidDrive;
    //this.finishCommand = finishCommand;
    yawPID = new PIDController(0.01, 0, 0);
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    error = distance-dt.getDisplacement();
    yawPID.setSetpoint(dt.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turningSpeed = yawPID.calculate(dt.getAngle());
    dt.tankDrive(speed + turningSpeed, speed - turningSpeed);
    //dt.tankDrive(speed, speed);
  
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
      if(Math.signum(distance)*1<0){
        return distance> dt.getDisplacement();
      }
      return distance< dt.getDisplacement();
  }
}
