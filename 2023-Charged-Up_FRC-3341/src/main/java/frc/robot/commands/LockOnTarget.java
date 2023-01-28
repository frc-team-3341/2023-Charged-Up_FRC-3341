// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LockOnTarget extends CommandBase {
  /** Creates a new LockOnTarget. */

private final Limelight lime;
private final Drivetrain drive;
private PIDController pid;


  public LockOnTarget(Limelight lime, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = lime;
    this.drive = drive;



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    PIDController pid = new PIDController(kP, kI, kD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("TV", RobotContainer.getLime().get_tv());
    SmartDashboard.putNumber("X", RobotContainer.getLime().get_tx());
    SmartDashboard.putNumber("Y", RobotContainer.getLime().get_ty());

    if(Limelight.get_tv() == 1){
kylian m'bappe
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}