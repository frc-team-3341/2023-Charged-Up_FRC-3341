// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Arm extends SubsystemBase {

  TalonSRX armTalon = new TalonSRX(Constants.OperatorConstants.armPort);
  TalonSRX extendingTalon = new TalonSRX(Constants.OperatorConstants.extPort);

  // Creates necessary Shuffleboard tab, visible on DriverStation as well
  private ShuffleboardTab pidTab = Shuffleboard.getTab("Arm PID");
  private GenericEntry armPID_P = pidTab.add("Arm PID P", Constants.PIDConstants.armPID_P).getEntry();
  private GenericEntry armPID_I = pidTab.add("Arm PID I", Constants.PIDConstants.armPID_I).getEntry();
  private GenericEntry armPID_D = pidTab.add("Arm PID D", Constants.PIDConstants.armPID_D).getEntry();

  // Create data log entry for angle
  public DoubleLogEntry angleLog;

  // Creates a PID controller from GenericEntry NetworkTables entries
  public PIDController armPID = new PIDController(
  armPID_P.getDouble(Constants.PIDConstants.armPID_P), 
  armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
  armPID_D.getDouble(Constants.PIDConstants.armPID_D));

  public double targetAngle = 0.0;

  // Manual override
  // True = Semi-Auto (default), False = Manual
  public boolean override = true;

  // Data logging override
  // True = Logging, False = Not Logging (default)
  public boolean logOverride = false;

  public double differenceInAngle = 10;

  /** Creates a new Arm. */
  public Arm() {
    // Reset encoders to either 0 or otherwise an arbitrary offset
    resetEncoders();
    armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    extendingTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Sets max current limit for amperage
    armTalon.configPeakCurrentLimit(15);

   // extendingTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
   extendingTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
   extendingTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
   
    //armTalon.setSensorPhase(true);
    //extendingTalon.setInverted(true);
    armTalon.setInverted(true);

    // Very important - sets max output, so that PID doesn't output an output that it too high
    armTalon.configPeakOutputForward(0.4);
    armTalon.configPeakOutputReverse(-0.4);

    //sets limits for how far the talon can extend forwards and backwards
    armTalon.configForwardSoftLimitThreshold(Constants.Measurements.upperAngleBound*Constants.Measurements.degreesToTicks);
    armTalon.configReverseSoftLimitThreshold(Constants.Measurements.lowerAngleBound*Constants.Measurements.degreesToTicks);

    // Sets soft limits on extension
    extendingTalon.configForwardSoftLimitThreshold(0);
    extendingTalon.configReverseSoftLimitThreshold(31000);

    // Reset Extending Talon
    extendingTalon.setSelectedSensorPosition(0);

    // Tolerance from reaching the setpoint
    armPID.setTolerance(2);

    // Integration range, if we are using the integral term of PID
    armPID.setIntegratorRange(-5, 5);

    // Only use the datalogger in testing!

    // Starts datalogger
    DataLogManager.start();

    // Set up data log entries
    DataLog log = DataLogManager.getLog();
    angleLog = new DoubleLogEntry(log, "/my/double");
  }

  // Tested 1/21/2022
  /**
  // The testing arm holds with the power of the calculated feedforward, not the action (manual increment of 20%).
  // This is calculated with sine, where the max voltage is 1.12/12 = 9% voltage
  */
  public void moveArm(double power){
    armTalon.set(ControlMode.PercentOutput, power*0.2+(Math.sin(getAngle())*(1.12/12.0)));
  }
  public void extendArm(double power){
    //extendingTalon.set(ControlMode.PercentOutput, power);

    if(!(extendingTalon.getSelectedSensorPosition()<Constants.Measurements.lowerScrewBound || extendingTalon.getSelectedSensorPosition()>Constants.Measurements.upperScrewBound)){
      extendingTalon.set(ControlMode.PercentOutput, power);
    }
    else if(extendingTalon.getSelectedSensorPosition()<Constants.Measurements.lowerScrewBound){
      extendingTalon.set(ControlMode.PercentOutput, 0.1);
    }
    else if(extendingTalon.getSelectedSensorPosition()>Constants.Measurements.upperScrewBound){
      extendingTalon.set(ControlMode.PercentOutput, -0.1);
    }
  }

  /**
  The robot starts at 0 degrees, and drops down to 90 degrees for the horizontal position.
  Since sin(90) = 1, then we can multiply it by the max holding voltage.
  Max holding voltage for testing pivot: 1.12 volts = 0.09 power.
  */
  public void resetEncoders() {
    armTalon.setSelectedSensorPosition(0, 0, 10);
  }

  // Returns the angle of the Arm
  public double getAngle() {
    double result = (armTalon.getSelectedSensorPosition(0)/4096.0)*360.0;
    return result;
  }

  public double getLeadScrewPos() {
    double result = (armTalon.getSelectedSensorPosition(0)/4096.0)*Constants.Measurements.gearRatio*Constants.Measurements.threadLength;
    return result;
  }

  /**
  Updates the PID loop with a feedforward value
   */
  public void updatePID() {
    // Without two clamps: 1.12
    // With clamps: 1.2
    armTalon.set(ControlMode.PercentOutput, armPID.calculate(getAngle())+(Math.sin(getAngle())*(1.4/12.0)));
    SmartDashboard.putNumber("PID: ", armPID.calculate(getAngle())+(Math.sin(getAngle())*(1.4/12.0)));
  }

  /**
  Whether the PID controller is at the setpoint.
   */
  public boolean withinSetpoint() {
    return armPID.atSetpoint();
  }

  /** 
  Sets subsystem's internal angle
   */
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  public void setDifferenceInAngle(double diff) {
    this.differenceInAngle = diff;
  }

  @Override
  public void periodic() {
    if (RobotContainer.getJoy1().getRawButtonReleased(2)) {
      override = !override;
    }

    if (RobotContainer.getJoy1().getRawButtonReleased(12)) {
      logOverride = !logOverride;
    }

    if (override) {
      moveArm(-1.0*RobotContainer.getJoy1().getY());
      extendArm(-1.0*RobotContainer.getJoy1().getX());

      // We want to reset the target angle to the current angle
      targetAngle = getAngle();
    } else if (!override) {
      armPID.setSetpoint(targetAngle);
      updatePID();
    }

    if (logOverride) {
      angleLog.append(getAngle());
    }

    SmartDashboard.putNumber("Ticks: ", armTalon.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Voltage: ", armTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle: ", getAngle());
    SmartDashboard.putNumber("Angle for Graph: ", getAngle());
    SmartDashboard.putNumber("Joystick Position: ", RobotContainer.getJoy1().getX());

    // Repeatedly set new PID constants from Driverstation
    if(differenceInAngle >= 0) {
    armPID.setPID(
      armPID_P.getDouble(0.70/Math.abs(differenceInAngle)), 
      armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
      armPID_D.getDouble(Constants.PIDConstants.armPID_D));
    }
    if(differenceInAngle <= 0) {
      armPID.setPID(
        armPID_P.getDouble(0.70/Math.abs(differenceInAngle)), 
        armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
        armPID_D.getDouble(Constants.PIDConstants.armPID_D));
      }
    
    if(!override && RobotContainer.getJoy1().getY()>0 && RobotContainer.getJoy1().getTriggerPressed() && targetAngle <= Constants.Measurements.upperAngleBound){
      SmartDashboard.putString("Status: ", "Incrementing");
      targetAngle += 10;
    }
    else if(!override && RobotContainer.getJoy1().getY()<0 && RobotContainer.getJoy1().getTriggerPressed() && targetAngle >= Constants.Measurements.lowerAngleBound){
      SmartDashboard.putString("Status: ", "Decrementing");
      targetAngle -= 10;
    } else {
      SmartDashboard.putString("Status: ", "None");
    }
    SmartDashboard.putNumber("kP: ", 0.4/differenceInAngle);
    SmartDashboard.putNumber("kI: ", armPID_I.getDouble(Constants.PIDConstants.armPID_I));
    SmartDashboard.putNumber("Lead Screw Raw Pos: ", extendingTalon.getSelectedSensorPosition());
  }
}
