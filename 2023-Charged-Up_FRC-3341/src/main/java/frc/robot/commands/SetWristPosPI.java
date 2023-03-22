// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PoweredIntake;

public class SetWristPosPI extends CommandBase {
  
  private PoweredIntake PI;
  private double angle;

  /** Rotates the Wrist's servo motor(s) to a certain position
  * @param claw - Claw subsystem
  * @param angle - Desired angle in degrees (-900 deg to 900 deg)
  */
  Timer timer;

  public SetWristPosPI(PoweredIntake PI, double angle) {
    this.PI = PI;
    this.angle = angle;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the servo position to the value of the target angle plus 180
    // Since 0 degrees of input is at 180 degrees on the servo, then we have to add 180 in the subsystem
    PI.setWristServoPos(angle);
    SmartDashboard.putString("Current Command: ", "SetWristPosPI");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1;
  }
}
