// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoweredIntake;

public class SpitTap extends CommandBase {

  PoweredIntake poweredIntake;
  double power;
  Timer timer;
  int taps;

  public SpitTap(PoweredIntake poweredIntake, double power, int taps) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.poweredIntake = poweredIntake;
    this.power = power;
    this.taps = taps;
    timer = new Timer();
    addRequirements(poweredIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (taps > 0){
      if(timer.get() < 0.05){
        poweredIntake.setFlywheelPower(power);
      }else if(timer.get() > 0.2){
        timer.reset();
        taps--;
      }else if(timer.get() > 0.05){
        poweredIntake.setFlywheelPower(0);
      }
    }
    SmartDashboard.putString("Current Command: ", "SpitTap");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    poweredIntake.setFlywheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return taps == 0;
  }
}
