// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Claw extends SubsystemBase {

  Servo clawServo = new Servo(Constants.OperatorConstants.clawServoPort);
  Servo wristServo = new Servo(Constants.OperatorConstants.wristServoPort);

  public boolean clawed = false;
  private double angle = 0.0;
  // Timer for controls
  private Timer controlsTimer = new Timer();
  
  /** Creates a new Claw. */
  // Claw either has 3 talons or 1 - still TBD
  public Claw() {
    controlsTimer.reset();
  }

  /**
  * Moves the Claw to a certain position
  * @param angle - Position to move the servo (0 deg to 1800 deg)
  */
  public void setClawServoPos(double angle) {
    // The servo turns 5 rotations per 1 value units given
    double pos = ((angle/360.0))/5.0;
    clawServo.setPosition(pos);
  }

  /**
  * Gets the Claw's position
  */
  public double getClawServoPos() {
    double pos = (clawServo.getPosition()*5.0)*360.0;
    return pos;
  }

  /**
   * Moves the Wrist to a certain position
   * @param angle - Position to move the servo (-900 deg to 900 deg)
   */
  public void setWristServoPos(double angle) {
    // The servo turns 5 rotations per 1 value units given
    // We add 2.5 to convert it to the scale of the middlemost value being the center (2.5 rotations)
    double pos = (((angle)/360.0) + 2.5)/5.0;
    wristServo.setPosition(pos);
  }

  /**
  * Gets the Wrist's position
  */
  public double getWristServoPos() {
    double pos = (wristServo.getPosition()*5.0 - 2.5)*360.0;
    return pos;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Manually increments and decrements the angle of the Claw
    // This is done when the buttons are pressed, every certain number of seconds
    
    if (RobotContainer.getJoy1().getRawButton(Constants.ButtonMap.clawDecrement) && angle > 0.0 && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
      angle -= 5.0;
      setClawServoPos(angle);
      controlsTimer.reset();
    }
    else if (RobotContainer.getJoy1().getRawButton(Constants.ButtonMap.clawIncrement) && angle < Constants.Measurements.servoAngleLimit && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
      angle += 5.0;
      setClawServoPos(angle);
      controlsTimer.reset();
    } else if (controlsTimer.get() >= Constants.ButtonMap.controlsDelay) {
      controlsTimer.reset();
    }

    SmartDashboard.putNumber("Claw Servo Position", clawServo.getPosition());
    SmartDashboard.putNumber("Claw Servo Angle:", angle);
  }
}