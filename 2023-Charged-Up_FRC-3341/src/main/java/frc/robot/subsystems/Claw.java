// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  TalonSRX clawTalon = new TalonSRX(Constants.OperatorConstants.clawPort);
  TalonSRX wristTalon = new TalonSRX(Constants.OperatorConstants.wristPort);
  TalonSRX flyOneTalon = new TalonSRX(Constants.OperatorConstants.flyWheelOnePort);
  TalonSRX flyTwoTalon = new TalonSRX(Constants.OperatorConstants.flyWheelTwoPort);
  // Talons are placeholders until we know the final claw design

  Servo leftServo = new Servo(Constants.OperatorConstants.leftServoPort);

  boolean clawed = false;
  private double angle = 0.0;
  // Claw either has 3 talons or 1 - still TBD
  public Claw() {
    clawTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    wristTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }

  public double getClawPos(){
    return clawTalon.getSelectedSensorPosition();
  }

  public double getWristAngle(){
    return wristTalon.getSelectedSensorPosition();
  }

  public void pinchManual(double power){
    clawTalon.set(ControlMode.PercentOutput, power);
  }

  public void setClawServoPos(double angle) {
    // The servo turns 5 rotations per 1 value units given
    double pos = ((angle/360.0))/5.0;
    leftServo.set(pos);
  }

  public double getClawServoPos() {
    double pos = (leftServo.getPosition()*5.0)*360.0;
    return pos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Manually increments and decrements the angle of the Claw
    if (RobotContainer.getJoy1().getRawButtonPressed(Constants.ButtonMap.clawDecrement) && angle >= 0.0) {
      angle -= 5.0;
       leftServo.setAngle(angle);
    }
    else if (RobotContainer.getJoy1().getRawButtonPressed(Constants.ButtonMap.clawIncrement) && angle <= Constants.Measurements.servoAngleLimit) {
      angle += 5.0;
      leftServo.setAngle(angle);
    }

    SmartDashboard.putNumber("Voltage", clawTalon.getMotorOutputVoltage());
    SmartDashboard.putNumber("Servo Position", leftServo.getPosition());
    SmartDashboard.putNumber("Desired Angle:", angle);
  }
}