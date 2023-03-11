// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


public class RobotContainer {
  public static final Joystick joystick0 = new Joystick(0);
  public static final Joystick joystick1 = new Joystick(1);
  public static final Joystick joystick2 = new Joystick(2);
  // private final TankDrive tankDrive;
  private static DriveTrain dt;
  
  // The robot's subsystems and commands are defined here...


  public final Arm arm = new Arm();
  public final Claw claw = new Claw();
  public final StarClaw starClaw = new StarClaw();

  // final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureBindings();
    // Configure the button bindings
    dt = new DriveTrain();
    // tankDrive = new TankDrive(dt, joystick0, joystick1);
    

  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureBindings() {
    JoystickButton triggerStowPos = new JoystickButton(joystick0, Constants.ButtonMap.stowPosition);
    triggerStowPos.onTrue(new Rotate(arm, 0));

    JoystickButton triggerMiddlePos = new JoystickButton(joystick0, Constants.ButtonMap.middlePosition);
    triggerMiddlePos.onTrue(new Rotate(arm, 60));

    JoystickButton triggerOtherPos = new JoystickButton(joystick0, Constants.ButtonMap.otherArbPosition);
    triggerOtherPos.onTrue(new Rotate(arm,50));

    JoystickButton triggerGroundPos = new JoystickButton(joystick0, Constants.ButtonMap.groundPosition);
    triggerGroundPos.onTrue(new Rotate(arm, 90));

    // JoystickButton triggerExt = new JoystickButton(leftJoystick, Constants.ButtonMap.fullyExtendedArm);
    // Extends 1 inch for testing
    // triggerExt.onTrue(new Extend(arm, 1));

    // JoystickButton RestPosExt = new JoystickButton(leftJoystick, Constants.ButtonMap.restPositionArm);
    // RestPosExt.onTrue(new Extend(arm, 0));

    /*JoystickButton triggerWristRight = new JoystickButton(joystick1, Constants.ButtonMap.wristRight);
    triggerWristRight.onTrue(new SetWristPos(claw, 225));

    JoystickButton triggerWristLeft = new JoystickButton(joystick1, Constants.ButtonMap.wristLeft);
    triggerWristLeft.onTrue(new SetWristPos(claw, -225));

    JoystickButton triggerWristCenter = new JoystickButton(joystick1, Constants.ButtonMap.wristCenter);
    triggerWristCenter.onTrue(new SetWristPos(claw, 0));

    
    JoystickButton triggerClawRest = new JoystickButton(joystick1, Constants.ButtonMap.clawRest);
    triggerClawRest.onTrue(new SetClawPos(claw, 0));

    JoystickButton triggerClawClosed = new JoystickButton(joystick1, Constants.ButtonMap.clawClosed);
    triggerClawClosed.onTrue(new SetClawPos(claw, 50));
    */

   
    JoystickButton triggerClawRest = new JoystickButton(joystick1, Constants.ButtonMap.clawRest);
    triggerClawRest.onTrue(new SetStarClawPos(starClaw, 0));

    JoystickButton triggerClawClosed = new JoystickButton(joystick1, Constants.ButtonMap.clawClosed);
    triggerClawClosed.onTrue(new SetStarClawPos(starClaw, 2));

    JoystickButton triggerFlywheel = new JoystickButton(joystick1, Constants.ButtonMap.flywheelOn);
    triggerClawClosed.onTrue(new SetStarClawFlywheel(starClaw, 0.4));
    triggerClawClosed.onFalse(new SetStarClawFlywheel(starClaw, 0.0));
    

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public static Joystick getJoy1() {
    return joystick0;
  }

  public static Joystick getJoy2() {
    return joystick1;
}

public static Joystick getJoy3() {
  return joystick2;
}
  
   public static DriveTrain getDriveTrain(){
    return dt;
  }
}



