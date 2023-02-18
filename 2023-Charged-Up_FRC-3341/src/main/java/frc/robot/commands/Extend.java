// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class Extend extends CommandBase {
  /** Creates a new Extend. */
  public Arm arm;
  private double length;
  private double direction = 1.0;
  public Extend(Arm arm, double length) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.length = length;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (length >= arm.getLeadScrewPos()) {
      direction = 1.0;
    } else if (length <= arm.getLeadScrewPos()) {
      direction = -1.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.extendArm(0.3*direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.extendArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(direction < 0 && length>= arm.getLeadScrewPos()){
      return true;
    }
    if(direction > 0 && length<= arm.getLeadScrewPos()){
      return true;
    }
    return false;
  }
}
