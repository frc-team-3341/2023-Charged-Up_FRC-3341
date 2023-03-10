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
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  // Creates Arm and Extending talons
  TalonSRX rotTalon = new TalonSRX(Constants.OperatorConstants.armPort);
  TalonSRX extendingTalon = new TalonSRX(Constants.OperatorConstants.extPort);

  // Creates necessary Shuffleboard tab, visible on DriverStation as well
  private ShuffleboardTab pidTab = Shuffleboard.getTab("Arm PID");
  private GenericEntry armPID_P = pidTab.add("Arm PID P", Constants.PIDConstants.armPID_P).getEntry();
  private GenericEntry armPID_I = pidTab.add("Arm PID I", Constants.PIDConstants.armPID_I).getEntry();
  private GenericEntry armPID_D = pidTab.add("Arm PID D", Constants.PIDConstants.armPID_D).getEntry();
  private GenericEntry armPID_K = pidTab.add("Power Const", Constants.PIDConstants.armPID_K).getEntry();

  // Create data log entry for angle
  public DoubleLogEntry angleLog;

  // Creates a PID controller from GenericEntry NetworkTables entries
  public PIDController armPID = new PIDController(
  armPID_P.getDouble(Constants.PIDConstants.armPID_P), 
  armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
  armPID_D.getDouble(Constants.PIDConstants.armPID_D));

  // Creates target angle variable
  private double targetAngle = 0.0;

  // Creates variable for local PID Power Constant (K)
  // This is used to calculate the kP from the difference in angle between the original and target positions
  // Example: K/difference = kP = 0.7/45.0 = 0.01555
  public double powerConstant = armPID_K.getDouble(Constants.PIDConstants.armPID_K);

  // Manual override
  // True =  Manual Arm control (default), False = Auto
  public static boolean override = true;

  // Data logging override
  // True = Logging, False = Not Logging (default)
  public boolean logOverride = false;

  // Default difference in angle in order to calculate kP
  // Its value is always changed in the subsystem, but shouldn't be 0!
  public double differenceInAngle = 10.0;

  /** Creates a new Arm. */
  public Arm() {
    // Reset encoders to either 0 or otherwise an arbitrary offset
    resetEncoders();
    rotTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    extendingTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Sets max current limit for amperage
    rotTalon.configPeakCurrentLimit(15);

    // Important for future use on final mechanism - configures limit switches for extension
    extendingTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    extendingTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

    rotTalon.setInverted(true);

    // Very important - sets max output, so that PID doesn't output an output that it too high
    // Might be changed based upon Arm's torque and sensitivity to voltage
    rotTalon.configPeakOutputForward(0.4);
    rotTalon.configPeakOutputReverse(-0.4);

    // Sets limits for how far the talon can extend forwards and backwards
    rotTalon.configForwardSoftLimitThreshold(Constants.Measurements.upperAngleBound*Constants.Measurements.degreesToTicks);
    rotTalon.configReverseSoftLimitThreshold(Constants.Measurements.lowerAngleBound*Constants.Measurements.degreesToTicks);
    extendingTalon.configForwardSoftLimitThreshold(((Constants.Measurements.upperScrewBound*Constants.Measurements.gearRatio)/Constants.Measurements.threadLength)*4096.0);
    extendingTalon.configReverseSoftLimitThreshold(((Constants.Measurements.lowerScrewBound*Constants.Measurements.gearRatio)/Constants.Measurements.threadLength)*4096.0);

    // NOTICE: These lines of code are needed to enable the limits!
    rotTalon.configForwardSoftLimitEnable(true);
    rotTalon.configReverseSoftLimitEnable(true);
    extendingTalon.configForwardSoftLimitEnable(true);
    extendingTalon.configReverseSoftLimitEnable(true);

    // Reset Extending Talon
    extendingTalon.setSelectedSensorPosition(0);

    // Tolerance from reaching the setpoint
    armPID.setTolerance(2);

    // Integration range, if we are using the integral term of PID
    // Meaning that the controller only takes the integral when it
    // is within a certain range of the setpoint
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
  * The testing arm holds with the power of the calculated feedforward, not the action (manual increment of 20%).
  * This is calculated with sine, where the max voltage is 1.12/12 = 9% voltage
  * We might need to increase the multiplier on the input, so that when operating manually, the pivot works better for the driver
  @param power - Percent input into the controller
  */
  public void rotateArm(double power) {
      rotTalon.set(ControlMode.PercentOutput, power*0.4+(Math.sin(getAngle())*(Constants.PIDConstants.armManualHoldingVoltage/12.0))); 
  }
  public void extendArm(double power) {
    extendingTalon.set(ControlMode.PercentOutput, power);
  }

  /**
  The robot starts at 0 degrees, and drops down to 90 degrees for the horizontal position.
  Since sin(90) = 1, then we can multiply it by the max holding voltage.
  Max holding voltage for testing pivot: 1.12 volts = 0.09 power.
  */
  public void resetEncoders() {
    rotTalon.setSelectedSensorPosition(0, 0, 10);
  }

  /** 
  Returns the angle of the Arm (degrees)
  */
  public double getAngle() {
    double result = (rotTalon.getSelectedSensorPosition(0)/4096.0)*360.0;
    return result;
  }

  /** 
  Returns the position of the Leadscrew (inches)
  */
  public double getLeadScrewPos() {
    double result = ((extendingTalon.getSelectedSensorPosition(0)/4096.0)*Constants.Measurements.threadLength)/Constants.Measurements.gearRatio;
    return result;
  }

  /**
  Updates the PID loop with a calculated feedforward value
   */
  public void updatePID() {
    // Without two clamps: 1.12
    // With clamps: 1.2
    rotTalon.set(ControlMode.PercentOutput, armPID.calculate(getAngle())+(Math.sin(getAngle())*(Constants.PIDConstants.armHoldingVoltage/12.0)));
    SmartDashboard.putNumber("PID: ", armPID.calculate(getAngle())+(Math.sin(getAngle())*(Constants.PIDConstants.armHoldingVoltage/12.0)));
  }

  /**
  Whether the PID controller is at the setpoint.
   */
  public boolean withinSetpoint() {
    return armPID.atSetpoint();
  }

  /** 
  Sets subsystem's internal angle
  @param angle - Angle to set the arm
   */
  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  /** 
  Sets subsystem's internal difference in angle, in order to calculate kP
  @param diff - Difference in angle
   */
  public void setDifferenceInAngle(double diff) {
    this.differenceInAngle = diff;
  }
  
  /** 
  Gets the status of control (manual is true, auto is false)
  */
  public static boolean getOverride() {
    return override;
  }
  /**
  Sets the status of control (manual is true, auto is false).
  Useful for auto commands.
  @param o - Boolean for overriding to manual mode
  */
  public void setOverride(boolean o) {
    override = o;
  }

  // Runs periodically  
  @Override
  public void periodic() {

    // Button to toggle between manual and PID control
    if (RobotContainer.getJoy1().getRawButtonReleased(Constants.ButtonMap.manualOverride)) {
      override = !override;
    }

    // Button to toggle whether or not to log
    if (RobotContainer.getJoy1().getRawButtonReleased(Constants.ButtonMap.logButton)) {
      logOverride = !logOverride;
    }

    if (RobotContainer.getJoy1().getPOV() == 0) {
      // Move extension forward at POV pos of 0
      extendArm(0.4);
    }  else if (RobotContainer.getJoy1().getPOV() == 180) {
      // Move extension backward at POV pos of 180
      extendArm(-0.4);
    }
    else {
      extendArm(0);
    }

    // If controlling manually
    if (override) {
      // We want to reset the target angle to the current angle
      rotateArm(RobotContainer.getJoy1().getY());
      targetAngle = getAngle();
    // Else if not controlling manually
    } else if (!override) {
      armPID.setSetpoint(targetAngle);
      updatePID();
    }

    // Override for logging button (REMOVE IN FINAL CODE)
    if (logOverride) {
      angleLog.append(getAngle());
    }

    // Repeatedly set new Power constant for finding kP
    powerConstant = armPID_K.getDouble(Constants.PIDConstants.armPID_K);

    // Repeatedly set new PID constants from DriverStation
    // If the Direction of the Arm is greater than 0, set certain PID constants
    if (differenceInAngle >= 0) {
    armPID.setPID(
      armPID_P.getDouble(powerConstant/Math.abs(differenceInAngle)), 
      armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
      armPID_D.getDouble(Constants.PIDConstants.armPID_D));
    }
    // If the Direction of the Arm is less than 0, then set certain PID constants
    else if (differenceInAngle <= 0) {
      armPID.setPID(
        armPID_P.getDouble(powerConstant/Math.abs(differenceInAngle)), 
        armPID_I.getDouble(Constants.PIDConstants.armPID_I), 
        armPID_D.getDouble(Constants.PIDConstants.armPID_D));
    }
    
    // Increment arm angle if the joystick's y axis is at a positive value, the trigger button is pressed, and the target angle is below a certain bound
    if (!override && RobotContainer.getJoy1().getY() > 0 && RobotContainer.getJoy1().getTriggerPressed() && targetAngle <= Constants.Measurements.upperAngleBound) {
      SmartDashboard.putString("Status: ", "Incrementing");
      targetAngle += 10;
    // Decrement arm angle if the joystick's y axis is at a negative value, the trigger button is pressed, and the target angle is above a certain bound
    } else if (!override && RobotContainer.getJoy1().getY() < 0 && RobotContainer.getJoy1().getTriggerPressed() && targetAngle >= Constants.Measurements.lowerAngleBound) {
      SmartDashboard.putString("Status: ", "Decrementing");
      targetAngle -= 10;
    } else {
      SmartDashboard.putString("Status: ", "None");
    }

    // Enable/Disable Soft limits when the leadscrew extends a certain amount
    // This is so that it can stow safely, yet extend beyond the frame to pick up field elements
    if (getLeadScrewPos() >= Constants.Measurements.fullyExtendedLeadScrewThreshold) {
      rotTalon.configForwardSoftLimitThreshold(Constants.Measurements.bumperAngleBound*Constants.Measurements.degreesToTicks);
      rotTalon.configForwardSoftLimitEnable(true);
    } else if (getLeadScrewPos() <  Constants.Measurements.fullyExtendedLeadScrewThreshold) {
      rotTalon.configForwardSoftLimitThreshold(Constants.Measurements.upperAngleBound*Constants.Measurements.degreesToTicks);
      rotTalon.configForwardSoftLimitEnable(true);
    }

    // Prints status of certain things to the DriverStation
    SmartDashboard.putNumber("kP: ", 0.7/differenceInAngle);
    SmartDashboard.putNumber("kI: ", armPID_I.getDouble(Constants.PIDConstants.armPID_I));
    SmartDashboard.putNumber("Lead Screw Raw Pos: ", extendingTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Extension Pos In Inches: ", getLeadScrewPos());
    SmartDashboard.putNumber("POV Angle", RobotContainer.getJoy1().getPOV());
    SmartDashboard.putNumber("Arm Ticks: ", rotTalon.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Arm Voltage: ", rotTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Arm Angle: ", getAngle());
    SmartDashboard.putNumber("Joystick Y Position: ", RobotContainer.getJoy1().getY());
    
  }
}
