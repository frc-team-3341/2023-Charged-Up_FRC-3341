// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockOnTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Limelight lime;
  private final DriveTrain dt;
  private double target;
  private double error; 
  
  public LockOnTarget(Drivetrain drive, Limelight lime, double target) {
    this.drive = drive;
    this.lime = lime;
    this.target = target;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    error = target - Limelight.get_tx();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    error = target - Limelight.get_tx();
    double speed = .3*error/30;

      SmartDashboard.putNumber("Error", error);
      SmartDashboard.putNumber("LimelightTVNUM", RobotContainer.get_tv());
      SmartDashboard.putNumber("LimelightX", Limelight.get_tx());
      SmartDashboard.putNumber("LimelightY", RobotContainer.get_ty());

    if(Math.abs(speed) < .2){
      speed = .2 * Math.abs(error)/error;
    }
    dt.tankDrive(-speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // This is called when there is a target in sight, so the chassis will stop turning completely
    dt.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // This code should run forever until there is a valid target (reflective tape or yellow ball)
    //in sight and will continue to repeat
    return Math.abs(error) <= 1;
    
  }
}
