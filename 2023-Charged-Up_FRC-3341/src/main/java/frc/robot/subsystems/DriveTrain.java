// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;

  DifferentialDrive diffDrive;
  private double ticksToMeters = (127.0/10581.0)/100.0;

  
  public Drivetrain(){

    leftDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.RightDriveTalonPort);
    _leftDriveVictor = new VictorSPX(Constants.OperatorConstants.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.OperatorConstants.RightDriveVictorPort);

    _leftDriveVictor.follow(leftDriveTalon);
    _rightDriveVictor.follow(rightDriveTalon);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(true);
    rightDriveTalon.setInverted(false);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    
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

   diffDrive = new DifferentialDrive(rightDriveTalon, leftDriveTalon);

   rightDriveTalon.configPeakOutputForward(0.7);
   leftDriveTalon.configPeakOutputReverse(-0.7);


  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  public void arcadeDrive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }
  
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
    
  }
/* 
  public double getVelocity(double setPoint){
    PIDController pid = new PIDController(0.93825, 0.0, 0.0);
    pid.setSetpoint(setPoint);
    pid.calculate(setPoint);
    return setPoint;
  }
*/

  @Override
  public void periodic() {
    tankDrive(RobotContainer.getJoy1().getY()*-0.2, RobotContainer.getJoy2().getY()*-0.2);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static void set(double calculate) {
  }
}