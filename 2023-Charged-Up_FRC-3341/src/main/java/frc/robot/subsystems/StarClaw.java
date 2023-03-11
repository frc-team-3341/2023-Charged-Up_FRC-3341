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

public class StarClaw extends SubsystemBase {
  /** Creates a new StarClaw. */
  public Servo leftClawServo;
  public Servo rightClawServo;
  public TalonSRX leftFlywheel;
  public TalonSRX rightFlywheel;

  private double clawPosition = 0.0;

  // Timer for controls
  private Timer controlsTimer = new Timer();
  
  public StarClaw() {
    leftClawServo = new Servo(Constants.OperatorConstants.leftStarClawServo);
    rightClawServo = new Servo(Constants.OperatorConstants.rightStarClawServo);

    leftFlywheel = new TalonSRX(Constants.OperatorConstants.leftFlywheelPort);
    rightFlywheel = new TalonSRX(Constants.OperatorConstants.rightFlywheelPort);

    leftFlywheel.setInverted(true);
    rightFlywheel.setInverted(false);

    leftClawServo.set(0);
    rightClawServo.set(0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

    SmartDashboard.putNumber("Claw Raw Servo Position", (leftClawServo.get() + rightClawServo.get()) / 2.0);
    SmartDashboard.putNumber("Claw Servo Position:", clawPosition);
  }
}
