// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
//comment


public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private static final WPI_TalonSRX leftDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.LeftDriveTalonPort);
  private static final WPI_TalonSRX rightDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.RightDriveTalonPort);
  
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;

  
  public DriveTrain() 
  {
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    _leftDriveVictor = new VictorSPX(Constants.OperatorConstants.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.OperatorConstants.RightDriveVictorPort);

    _leftDriveVictor.follow(leftDriveTalon);
    _rightDriveVictor.follow(rightDriveTalon);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    // leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    // rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    //leftDriveTalon.configPeakCurrentLimit(20);
    //rightDriveTalon.configPeakCurrentLimit(20);

    //leftDriveTalon.enableCurrentLimit(true);
    //rightDriveTalon.enableCurrentLimit(true);
    
    // Week 4 Motion Magic
   leftDriveTalon.config_kP(0, 3, 10);
   leftDriveTalon.config_kI(0, 0, 10);
   leftDriveTalon.config_kD(0, 0, 10);
   leftDriveTalon.configMotionAcceleration(220, 10);
   leftDriveTalon.configMotionCruiseVelocity(400, 10);
   // If Left Velocity is 200
   // then Left Accel 120

   rightDriveTalon.config_kP(0, 3, 10);
   rightDriveTalon.config_kI(0, 0, 10);
   rightDriveTalon.config_kD(0, 0, 10);
   rightDriveTalon.configMotionAcceleration(200, 10);
   rightDriveTalon.configMotionCruiseVelocity(400, 10);
   // If  Right Velocity is 200
   // then Right Accel 90

  }

  /**
   * Sets the Talons in Coast mode
   */
  public static void setCoastMode() {
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);
    
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
    SmartDashboard.putNumber("left wheel speed", leftSpeed);
    SmartDashboard.putNumber("right wheel speed", rightSpeed);
  }

  
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }


  @Override
  public void periodic() {
    setCoastMode();

    SmartDashboard.putNumber("MaxSpeed: ", Constants.Measurements.maxDriveSpeed);
    double maxSpeed = SmartDashboard.getNumber("MaxSpeed: ", 0.4);
    Constants.Measurements.maxDriveSpeed = maxSpeed;

    tankDrive(Math.pow(Math.abs(RobotContainer.getJoy3().getY()), 1.8)*Math.signum(RobotContainer.getJoy3().getY())*-maxSpeed, Math.pow(Math.abs(RobotContainer.getJoy3().getThrottle()), 1.8)*Math.signum(RobotContainer.getJoy3().getThrottle())*-maxSpeed);

    SmartDashboard.putNumber("left motor current", leftDriveTalon.getStatorCurrent());
    SmartDashboard.putNumber("right motor current", rightDriveTalon.getStatorCurrent());
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}