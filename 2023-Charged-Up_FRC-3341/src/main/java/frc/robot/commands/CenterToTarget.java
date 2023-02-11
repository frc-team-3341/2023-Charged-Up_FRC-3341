// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CenterToTarget extends CommandBase {
  /** Creates a new CenterToTarget. */
  // Here I took everything from limelight subsystem and called it lime
  public Limelight lime;
  // Here is the middle of the x and y axis on the target
  public double centerx;
  public double centery;
  private final TankDrive tankDrive;
  private static Drivetrain drive;
  public PIDController pid;

  public CenterToTarget(Limelight lime, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    tankDrive = new TankDrive(drive, null, null);
    this.lime = lime;
    this.drive = drive;
    // Connects limelight subsystem to this command
    addRequirements(drive, lime);
    centerx = lime.get_tx();
    centery = lime.get_ty();
    pid = new PIDController(0.01, 0.0003, 0.001);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    centerx = lime.get_tx();
    centery = lime.get_ty();
    drive.resetEncoders();
    pid.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // You need this because it keeps calling the value instead of only once because
    // the robot is moving
    centerx = lime.get_tx();
    centery = lime.get_ty();
    // IMPORTANT - The screen of limelight it works with negatives(left side of 0)
    // and positives(right side of 0)
    if (RobotContainer.getJoy1().getRawButton(2)) {
      if (lime.get_tv() == 1) {
        if (lime.get_tx() >= 1) {
          drive.tankDrive(-0.2, 0.2);
        }
        if (lime.get_tx() <= -1) {
          drive.tankDrive(0.2, -0.2);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}