// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class Extend extends CommandBase {
  public Arm arm;
  private double position;
  private double direction = 1.0;

  /** Extends the Arm's extension to a certain position
   * @param arm - Arm subsystem
   * @param position - Desired position in inches
   */
  
  public Extend(Arm arm, double position) {
    this.arm = arm;
    this.position = position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set direction of arm, based upon current position, and desired position
    if (position >= arm.getLeadScrewPos()) {
      direction = 1.0;
    } else if (position <= arm.getLeadScrewPos()) {
      direction = -1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Extends the arm with some amount of power multiplied by a direction
    arm.extendArm(Constants.ButtonMap.extensionPower*direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the arm when interrupted
    arm.extendArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     If the direction of the arm's motion is negative, 
     and the arm's position is equal to or less than the 
     desired position, then stop.
    */
    if (direction < 0 && position >= arm.getLeadScrewPos()) {
      return true;
    }
    /*
     If the direction of the arm's motion is positive, 
     and the arm's position is equal to or greater than the 
     desired position, then stop.
    */
    if (direction > 0 && position <= arm.getLeadScrewPos()) {
      return true;
    } else {
      // Continue the motion
      return false;
    }
  }
}
