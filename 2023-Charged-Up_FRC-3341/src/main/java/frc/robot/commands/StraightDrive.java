// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class StraightDrive extends CommandBase {
  /** Creates a new StraightDrive. */
DriveTrain dt;
Joystick joy;
  public StraightDrive(DriveTrain dt, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.joy = joy;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double avgSpeed = (Math.abs(joy.getY()) + Math.abs(joy.getThrottle())) / -2.0;
    dt.tankDrive(avgSpeed * Math.signum(joy.getY()), avgSpeed * Math.signum(joy.getThrottle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !joy.getRawButton(Constants.ButtonMap.driveStraight);
  }
}
