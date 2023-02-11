// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  TalonSRX clawTalon = new TalonSRX(Constants.OperatorConstants.clawPinchPort);
  TalonSRX wristTalon = new TalonSRX(Constants.OperatorConstants.wristPort);
  TalonSRX flywheelOneTalon = new TalonSRX(Constants.OperatorConstants.flywheelOne);
  TalonSRX flywheelTwoTalon = new TalonSRX(Constants.OperatorConstants.flywheelTwo);

  /** Creates a new Claw. */
  public Claw() {
    clawTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  public void setClawPower(double power) {
    clawTalon.set(ControlMode.PercentOutput, power);
  }
  
  public double getClawPosition() {
    double result = (clawTalon.getSelectedSensorPosition(0)/4096.0)*360.0;
    return result;
  }

  public void setWristTalon(double power) {
    wristTalon.set(ControlMode.PercentOutput, power);
  }
  
  public double getWristPosition() {
    double result = (wristTalon.getSelectedSensorPosition(0)/4096.0)*360.0;
    return result;
  }

  public void setFlywheelTalonOne(double power) {
    flywheelOneTalon.set(ControlMode.PercentOutput, power);
  }

  public void setFlywheelTalonTwo(double power) {
    flywheelTwoTalon.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pincher Position: ", getClawPosition());
    // This method will be called once per scheduler run
  }
}
