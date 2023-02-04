// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
  DriveTrain dt;
  double angle;
  PIDController pidController = new PIDController(0.002, 0, 0);
  public AutoTurn(DriveTrain dt, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavX();

    pidController.setSetpoint(angle);
    pidController.setTolerance(1);
    dt.setPowerLimits(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(dt.getAngle());
    dt.tankDrive(-speed, speed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0, 0);
    dt.setPowerLimits(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
