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

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  
  private final AHRS navX;
  public boolean logOverride = false;
  public DoubleLogEntry anglelog;

  public boolean brakeOverride;
  
  public DriveTrain() 
  {
    navX = new AHRS(SPI.Port.kMXP);

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
   leftDriveTalon.config_kP(0, 0.7, 10);
   leftDriveTalon.config_kI(0, 0, 10);
   leftDriveTalon.config_kD(0, 0.05, 10);
   leftDriveTalon.configMotionAcceleration(200, 900);
   leftDriveTalon.configMotionCruiseVelocity(400, 10);
   // If Left Velocity is 200
   // then Left Accel 120

   rightDriveTalon.config_kP(0, 0.7, 10);
   rightDriveTalon.config_kI(0, 0, 10);
   rightDriveTalon.config_kD(0, 0.05, 10);
   rightDriveTalon.configMotionAcceleration(200, 900);
   rightDriveTalon.configMotionCruiseVelocity(400, 10);
   // If  Right Velocity is 200
   // then Right Accel 90

  }

  public void brake(){
    leftDriveTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveTalon.setNeutralMode(NeutralMode.Brake);
    _leftDriveVictor.setNeutralMode(NeutralMode.Brake);
    _rightDriveVictor.setNeutralMode(NeutralMode.Brake);
  }

  public void coast(){
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);
    _leftDriveVictor.setNeutralMode(NeutralMode.Coast);
    _rightDriveVictor.setNeutralMode(NeutralMode.Coast);
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
  
   public void setPowerLimits(double limit){
    leftDriveTalon.configPeakOutputForward(limit);
    leftDriveTalon.configPeakOutputReverse(limit);
    rightDriveTalon.configPeakOutputForward(limit);
    rightDriveTalon.configPeakOutputReverse(limit);

  }
  
    public void magicDrive(double displacement){
    leftDriveTalon.set(ControlMode.MotionMagic, Constants.OperatorConstants.tickstoMeters*displacement);
    rightDriveTalon.set(ControlMode.MotionMagic, Constants.OperatorConstants.tickstoMeters*displacement);
  }
  
    public void resetNavX(){
    navX.reset();
  }
  public double getTicksLeft() {
    return leftDriveTalon.getSelectedSensorPosition();
  }

  public double getDisplacementLeft(){
    return (getTicksLeft() * Constants.OperatorConstants.tickstoMeters);
  }

  public double getTicksRight() {
    return rightDriveTalon.getSelectedSensorPosition();
  }

  public double getDisplacementRight(){
    return (getTicksRight() * Constants.OperatorConstants.tickstoMeters);
  }

  public double getDisplacement(){
    return (getDisplacementLeft() + getDisplacementRight()) / 2.0;
  }

  public double getLeftSpeed(){
    return leftDriveTalon.getSelectedSensorVelocity() * 10 * Constants.OperatorConstants.tickstoMeters;
  }

  public double getRightSpeed(){
    return rightDriveTalon.getSelectedSensorVelocity() * 10 * Constants.OperatorConstants.tickstoMeters;
  }

  public double getYAngle(){
    return -1 * navX.getRoll();
  }

  public double getAngle(){
    return navX.getAngle();
  }


  @Override
  public void periodic() {
    //setCoastMode();

    SmartDashboard.putNumber("MaxSpeed: ", Constants.Measurements.maxDriveSpeed);
    double maxSpeed = SmartDashboard.getNumber("MaxSpeed: ", 0.8);
    Constants.Measurements.maxDriveSpeed = maxSpeed;

    if(RobotContainer.getJoy3().getRawButton(2)) Constants.endBrakeMode = true;
    else if(RobotContainer.getJoy3().getRawButton(1)) Constants.endBrakeMode = false;

    // Left Drive Input - whether or not we use an Xbox
    double leftDriveInput = RobotContainer.getJoy3().getY();
    // Left Drive Function - Limits power based upon exponential function (e^x)
    double leftDriveFunction = Math.pow(Math.abs(leftDriveInput), 1.2)*Math.signum(leftDriveInput)*-maxSpeed;

    // Right Drive Input - whether or not we use an Xbox
    double rightDriveInput = RobotContainer.getJoy3().getThrottle();
    // Right Drive Function - Limits power based upon exponential function (e^x)
    double rightDriveFunction =  Math.pow(Math.abs(rightDriveInput), 1.2)*Math.signum(rightDriveInput)*-maxSpeed;

    if(!RobotContainer.getJoy3().getRawButton(Constants.ButtonMap.driveStraight)) this.tankDrive(leftDriveFunction, rightDriveFunction);

    // tankDrive(Math.pow(Math.abs(RobotContainer.getJoy3().getY()), 1.2)*Math.signum(RobotContainer.getJoy3().getY())*-maxSpeed, Math.pow(Math.abs(RobotContainer.getJoy3().getThrottle()), 1.2)*Math.signum(RobotContainer.getJoy3().getThrottle())*-maxSpeed);

    SmartDashboard.putNumber("left motor current", leftDriveTalon.getStatorCurrent());
    SmartDashboard.putNumber("right motor current", rightDriveTalon.getStatorCurrent());
    SmartDashboard.putNumber("right motor joystick", Math.pow(Math.abs(RobotContainer.getJoy3().getY()), 1.2)*Math.signum(RobotContainer.getJoy3().getY())*-maxSpeed);
    SmartDashboard.putNumber("left motor joystick",  Math.pow(Math.abs(RobotContainer.getJoy3().getThrottle()), 1.2)*Math.signum(RobotContainer.getJoy3().getThrottle())*-maxSpeed);

    SmartDashboard.putNumber("Yaw Angle: ", getAngle());
    SmartDashboard.putNumber("Y Angle: ", getYAngle());
    SmartDashboard.putNumber("Displacement: ", getDisplacement());
    SmartDashboard.putNumber("LeftSpeed: ", getLeftSpeed());
    SmartDashboard.putNumber("RightSpeed: ", getRightSpeed());
    SmartDashboard.putData("Chooser:", RobotContainer.getChooser());
    SmartDashboard.putString("Current Command:", RobotContainer.getAutonomousCommand().toString());
    SmartDashboard.putBoolean("Brake Mode: ", !Constants.endBrakeMode);
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
