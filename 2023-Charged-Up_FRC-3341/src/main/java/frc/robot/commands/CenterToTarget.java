// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
private final Drivetrain drive; 
private double speed; 
 private Joystick joystick1 = RobotContainer.getJoy1();
  public CenterToTarget(Limelight lime, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = lime; 
    this.drive = drive;
    // Connects limelight subsystem to this command
    addRequirements(lime);
    centerx = lime.get_tx();
    centery = lime.get_ty(); 
   // speed = 1 - Math.abs(0 - lime.get_tx());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    centerx = lime.get_tx();
    centery = lime.get_ty();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // You need this because it keeps calling the value instead of only once because the robot is moving 
    centerx = lime.get_tx();
   // Use this for arm to get the values for the height
    centery = lime.get_ty();
// IMPORTANT - The screen of limelight it works with negatives(left side of 0) and positives(right side of 0)
    if(centerx > 1) {
      drive.tankDrive(-0.2, 0.2);
    } else if(centerx < -1 ) {
      drive.tankDrive(-0.2, 0.2);
    } else {
      drive.tankDrive(0, 0);    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    //always make drivetrain stop in here
      drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     //return Math.abs(centerx) == 0;
    return joystick1.getRawButtonPressed(3);
  }
}
