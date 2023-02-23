// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StarClaw;

public class SetStarClawPos extends CommandBase {
  /** Creates a new SetStarClawPos. */

  public StarClaw claw;
  public double position;

  public SetStarClawPos(StarClaw claw, double position) {
    this.claw = claw;
    this.position = position;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    claw.setClawPos(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.getClawPos() == Math.abs(position);
  }
}
