// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {

  TalonSRX armTalon = new TalonSRX(Constants.OperatorConstants.armPort);

  public PIDController armPID = new PIDController(0.012, 0.0001, 0);

  public double angle = 0.0;

  public boolean override = true;

  /** Creates a new Arm. */
  public Arm() {
    resetEncoders();
    armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    //armTalon.setSensorPhase(true);
    armTalon.setInverted(true);

    // Very important - sets max output, so that PID doesn't output an output that it too high
    armTalon.configPeakOutputForward(0.4);
    armTalon.configPeakOutputReverse(-0.4);

    armPID.setTolerance(0.5);
    armPID.setIntegratorRange(-10, 10);
  }

  // Tested 1/21/2022
  // The testing arm holds with the power of the calculated feedforward, not the action (manual increment of 20%)
  // This is calculated with sine, where the max voltage is 1.12/12 = 9% voltage
  public void moveArm(double power){
    armTalon.set(ControlMode.PercentOutput, power*0.2+(Math.sin(getAngle())*(1.12/12.0)));
  }

  // The robot starts at 0 degrees, and drops down to 90 degrees for the horizontal position
  // Since sin(90) = 1, then we can multiply it by the max holding voltage
  // Max holding voltage for testing pivot - 1.12 volts = 0.09 power
  public void resetEncoders() {
    armTalon.setSelectedSensorPosition(0, 0, 10);
  }

  public double getAngle() {
    double result = (armTalon.getSelectedSensorPosition(0)/4096.0)*360.0;
    return result;
  }

  public void updatePID() {
    // Without two clamps: 1.12
    // With clamps: 1.2
    armTalon.set(ControlMode.PercentOutput, armPID.calculate(getAngle())+(Math.sin(getAngle())*(1.2/12.0)));
    SmartDashboard.putNumber("PID: ", armPID.calculate(getAngle())+(Math.sin(getAngle())*(1.2/12.0)));
  }

  public boolean withinSetpoint() {
    return armPID.atSetpoint();
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  @Override
  public void periodic() {
    if (RobotContainer.getJoy1().getRawButtonReleased(2)) {
      override = !override;
    }

    if (override) { // Auto code
      moveArm(-1.0*RobotContainer.getJoy1().getY());
      angle = 0.0;
    } else if (!override) {
      armPID.setSetpoint(angle);
      updatePID();
    }

    SmartDashboard.putNumber("Ticks: ", armTalon.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Voltage: ", armTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle: ", getAngle());
  
  }
}
