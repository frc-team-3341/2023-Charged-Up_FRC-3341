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
  Servo wristServo = new Servo(8);

  public boolean clawed = false;
  private double clawAngle = 0.0;
  // The wrist starts at 0.0 degrees, but is physically at 2.5 rotations
  private double wristAngle = 0.0;
  // Timer for controls
  private Timer controlsTimer = new Timer();
  
  /** Creates a new Claw. */
  // Claw either has 3 talons or 1 - still TBD
  public Claw() {
    controlsTimer.reset();
    // Sets Wrist to 0 degrees (resting position)
    clawServo.set(0.5);
   wristServo.set(0.5);
  }

  /**
  * Moves the Claw to a certain position
  * @param angle - Position to move the servo (0 deg to 1800 deg)
  */
  public void setClawServoPos(double angle) {
    // The servo turns 2 rotations per 1 value units given
    double pos = (angle/300.*Constants.Measurements.clawGearRatio);
    clawServo.setPosition(pos);
  }

  /**
  * Gets the Claw's position
  */
  public double getClawServoPos() {
    double pos = (clawServo.getPosition()*300.0/Constants.Measurements.clawGearRatio);
    return pos;
  }

  /**
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
    // Manually increments and decrements the angle of the Claw
    // This is done when the buttons are pressed, every certain number of seconds
    wristAngle = getWristServoPos();
    clawAngle = getClawServoPos();
    if (RobotContainer.getJoy2().getTrigger()) {
      // If POV is Down, then decrement
      if (RobotContainer.getJoy2().getPOV() == 0 && clawAngle > 0.0 && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
        clawAngle -= Constants.ButtonMap.clawIncrement;
        setClawServoPos(clawAngle);
        controlsTimer.reset();
      }
      // If POV is Up, then increment
      else if (RobotContainer.getJoy2().getPOV() == 180 && clawAngle < Constants.Measurements.clawAngleLimit && controlsTimer.get() <= Constants.ButtonMap.controlsDelay) {
        clawAngle += Constants.ButtonMap.clawIncrement;
        setClawServoPos(clawAngle);
        controlsTimer.reset();
      } else if (controlsTimer.get() >= Constants.ButtonMap.controlsDelay) {
        controlsTimer.reset();
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
    }

    SmartDashboard.putNumber("Claw Servo Position", clawServo.getPosition());
    SmartDashboard.putNumber("Claw Servo Angle:", getClawServoPos());
    SmartDashboard.putNumber("Wrist Servo Position", wristServo.getPosition());
    SmartDashboard.putNumber("Wrist Servo Angle:", wristAngle);
  }
}
