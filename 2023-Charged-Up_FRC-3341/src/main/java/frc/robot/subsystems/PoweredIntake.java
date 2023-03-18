// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PoweredIntake extends SubsystemBase {
  /** Creates a new StarClaw. */
  public Servo leftClawServo;
  public Servo rightClawServo;
  public Servo wristServo;
  public TalonSRX leftFlywheel;
  public TalonSRX rightFlywheel;

  private double clawPosition = 0.0;
  private double wristAngle = 0.0;

  // Timer for controls
  private Timer controlsTimer = new Timer();
  
  public PoweredIntake() {
    leftClawServo = new Servo(Constants.OperatorConstants.leftStarClawServo);
    rightClawServo = new Servo(Constants.OperatorConstants.rightStarClawServo);

    wristServo = new Servo(Constants.OperatorConstants.wristServoPort);

    leftFlywheel = new TalonSRX(Constants.OperatorConstants.leftFlywheelPort);
    rightFlywheel = new TalonSRX(Constants.OperatorConstants.rightFlywheelPort);

    leftFlywheel.setInverted(true);
    rightFlywheel.setInverted(false);

    leftClawServo.set(0);
    rightClawServo.set(0);
    wristServo.set(0.5);
    controlsTimer.reset();
    
  }

  public void setFlywheelPower(double power) {
    leftFlywheel.set(ControlMode.PercentOutput, power);
    rightFlywheel.set(ControlMode.PercentOutput, power);
  }

  public void setClawPos(double pos) {
    double posInRotations = (pos/Constants.Measurements.threadLengthStarClaw) / 5.0;
    leftClawServo.set(posInRotations);
    rightClawServo.set(posInRotations);
  }

  public double getClawPos() {
    double measurement = (rightClawServo.get() + leftClawServo.get())/2.0 * 5.0 * Constants.Measurements.threadLengthStarClaw;
    return measurement;
  }
  
  /*
   * Moves the Wrist to a certain position
   * @param angle - Position to move the servo (-225 deg to 225 deg)
   */
  public void setWristServoPos(double angle) {
    // The gear ratio of 4:1 is accounted for with the multiplication of 4
    // Meaning the range of the mechanism is from 225 to -225 degrees
    // The servo turns 5 rotations per 1 value units given
    // We add 2.5 to convert it to the scale of the middlemost value being the center (2.5 rotations)
    double pos = (((4*angle)/360.0) + 2.5)/5.0;
    wristServo.setPosition(pos);
  }

  /**
  * Gets the Wrist's position
  * Should be within the range of -225 to 225
  */
  public double getWristServoPos() {
    double pos = ((wristServo.getPosition()*5.0 - 2.5)*360.0)/4;
    return pos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristAngle = getWristServoPos();
    clawPosition = getClawPos();
    if (RobotContainer.getJoy2().getTrigger()) {
      // If POV is Down, then decrement
      if (RobotContainer.getJoy2().getPOV() == 180 && clawPosition > 0.0 && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
        clawPosition -= 0.25;
        setClawPos(clawPosition);
        controlsTimer.reset();
      }
      // If POV is Up, then increment
      else if (RobotContainer.getJoy2().getPOV() == 0 && clawPosition < Constants.Measurements.starClawPositionLimit && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
        clawPosition += 0.25;
        setClawPos(clawPosition);
        controlsTimer.reset();
      } else if (controlsTimer.get() >= Constants.ButtonMap.controlsDelay) {
        controlsTimer.reset();
      }
    }

    // If Y of joystick is Down, > 0, then decrement
    if (RobotContainer.getJoy2().getY() > 0.2 && wristAngle > Constants.Measurements.wristLowerLimit && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
      wristAngle -= Constants.ButtonMap.wristIncrement;
      setWristServoPos(wristAngle);
      controlsTimer.reset();
    }
    // If Y of joystick is Up, < 0, then increment
    else if (RobotContainer.getJoy2().getY() < -0.2 && wristAngle < Constants.Measurements.wristUpperLimit && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
      wristAngle += Constants.ButtonMap.wristIncrement;
      setWristServoPos(wristAngle);
      controlsTimer.reset();
    } else if (controlsTimer.get() >= Constants.ButtonMap.controlsDelay) {
      controlsTimer.reset();
    }

    SmartDashboard.putNumber("Claw Raw Servo Position", (leftClawServo.get() + rightClawServo.get()) / 2.0);
    SmartDashboard.putNumber("Claw Servo Position:", clawPosition);
    SmartDashboard.putNumber("Wrist Servo Position", wristServo.getPosition());
    SmartDashboard.putNumber("Wrist Servo Angle:", wristAngle);
  }
}
