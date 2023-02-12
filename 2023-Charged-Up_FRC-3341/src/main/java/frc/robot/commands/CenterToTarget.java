// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
  double speed = 0.0;
  
  

  public CenterToTarget(Limelight lime, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    tankDrive = new TankDrive(drive, null, null);
    this.lime = lime;
    this.drive = drive;
    // Connects limelight subsystem to this command
    addRequirements(drive, lime);
    centerx = Limelight.get_tx();
    centery = Limelight.get_ty();
    pid = new PIDController(0.0093825*2, 0.0, 0.0);
    pid.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    centerx = Limelight.get_tx();
    centery = Limelight.get_ty();
    drive.resetEncoders();
    pid.setSetpoint(0.0);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // You need this because it keeps calling the value instead of only once because
    // the robot is moving
    centerx = Limelight.get_tx();
    centery = Limelight.get_ty();
    speed = pid.calculate(centerx);

    if(Math.abs(speed) > 0.6){
      speed = Math.abs(0.6)*(Math.abs(speed)/speed);
    } else if(Math.abs(speed) < 0.15){
      speed = Math.abs(0.15)*(Math.abs(speed)/speed);
    }
      drive.tankDrive(speed, -speed);
    
      SmartDashboard.putNumber("Speed", speed);

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return pid.atSetpoint();
    return false;
  }
}