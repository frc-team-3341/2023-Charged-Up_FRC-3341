// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SetWristPos extends CommandBase {
  
  private Claw claw;
  private double angle;

  /** Rotates the Wrist's servo motor(s) to a certain position
  * @param claw - Claw subsystem
  * @param angle - Desired angle in degrees (-900 deg to 900 deg)
  */

  public SetWristPos(Claw claw, double angle) {
    this.claw = claw;
    this.angle = angle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set the servo position to the value of the target angle plus 180
    // Since 0 degrees of input is at 180 degrees on the servo, then we have to add 180 in the subsystem
    claw.setWristServoPos(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
