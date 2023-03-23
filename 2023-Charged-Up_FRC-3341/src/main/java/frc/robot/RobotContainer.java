// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoCone;
import frc.robot.commands.AutoCube;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.CenterToTarget;
import frc.robot.commands.Docking;
import frc.robot.commands.LockOnTarget;
import frc.robot.commands.MagicDrive;
import frc.robot.commands.Rotate;
import frc.robot.commands.SetPoweredClawFlywheel;
import frc.robot.commands.SetPoweredClawPos;
import frc.robot.commands.SetWristPosPI;
import frc.robot.commands.Stow;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoweredIntake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 
  //We have to initialize these objects for the SpinToTarget, ProtoTurret, and AutoTurret commands
  private final static Limelight lime = new Limelight();

  public static final Joystick joystick0 = new Joystick(0);
  public static final Joystick joystick1 = new Joystick(1);
  public static final Joystick joystick2 = new Joystick(2);
  private final TankDrive tankDrive;
  private static DriveTrain dt = new DriveTrain();
  
  private final static LockOnTarget lock = new LockOnTarget(dt, lime, 0);
  private final static CenterToTarget center = new CenterToTarget(lime, dt);
  public final Arm arm = new Arm();
  public final Claw claw = new Claw();
  public final PoweredIntake poweredIntake = new PoweredIntake();
  private final MagicDrive magicDrive;
  private final AutoTurn turn;
  private final AutoDrive forward;
  private final AutoBalance balance;
  private final Docking dock;

  private final AutoCone autoCone;
  private final AutoCube autoCube;
  private final Stow stow;

  // final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //dt = new DriveTrain();
    dt.resetEncoders();
    dt.coast();
    tankDrive = new TankDrive(dt, joystick0, joystick1);

    magicDrive = new MagicDrive(dt, 1.0);
    turn = new AutoTurn(dt, 90);
    forward = new AutoDrive(dt, 0, true);
    balance = new AutoBalance(dt);
    dock = new Docking(dt);
    autoCone = new AutoCone(dt, arm, poweredIntake);
    autoCube = new AutoCube(dt, arm, poweredIntake);
    stow = new Stow(dt, arm, poweredIntake);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton toTarget = new JoystickButton(joystick2, 4);
    toTarget.toggleWhenPressed( new CenterToTarget(lime, dt));
/* 
    JoystickButton triggerStowPos = new JoystickButton(joystick0, Constants.ButtonMap.stowPosition);
    triggerStowPos.onTrue(new Rotate(arm, 0));

    JoystickButton triggerMiddlePos = new JoystickButton(joystick0, Constants.ButtonMap.middlePosition);
    triggerMiddlePos.onTrue(new Rotate(arm, 70+5));

    JoystickButton triggerTopPos = new JoystickButton(joystick0, Constants.ButtonMap.topPosition);
    triggerTopPos.onTrue(new Rotate(arm,90+5));

    JoystickButton triggerGroundPos = new JoystickButton(joystick0, Constants.ButtonMap.groundPosition);
    triggerGroundPos.onTrue(new Rotate(arm, 16+5));*/

    // JoystickButton triggerExt = new JoystickButton(leftJoystick, Constants.ButtonMap.fullyExtendedArm);
    // Extends 1 inch for testing
    // triggerExt.onTrue(new Extend(arm, 1));

    // JoystickButton RestPosExt = new JoystickButton(leftJoystick, Constants.ButtonMap.restPositionArm);
    // RestPosExt.onTrue(new Extend(arm, 0));

    
    JoystickButton triggerWristRight = new JoystickButton(joystick1, Constants.ButtonMap.wristNinety);
    triggerWristRight.onTrue(new SetWristPosPI(poweredIntake, -112.5));

    //JoystickButton triggerWristLeft = new JoystickButton(joystick1, Constants.ButtonMap.wristOneEighty);
    //triggerWristLeft.onTrue(new SetWristPosPI(poweredIntake, -225));

    JoystickButton triggerWristCenter = new JoystickButton(joystick1, Constants.ButtonMap.wristCenter);
    triggerWristCenter.onTrue(new SetWristPosPI(poweredIntake, 0));

    /* 
    JoystickButton triggerClawRest = new JoystickButton(joystick1, Constants.ButtonMap.clawCube); //changed from joystick1 to joystick2
    triggerClawRest.onTrue(new SetClawPos(claw, 155/2)); //previously 155
    JoystickButton triggerClawClosed = new JoystickButton(joystick1, Constants.ButtonMap.clawCone);
    triggerClawClosed.onTrue(new SetClawPos(claw, 100/2)); //previously 100
    JoystickButton triggerClawOpen = new JoystickButton(joystick1, Constants.ButtonMap.clawOpen);
    triggerClawOpen.onTrue(new SetClawPos(claw, 245/2));
   */
   JoystickButton triggerStarClawRest = new JoystickButton(joystick0, Constants.ButtonMap.clawOpen);
    triggerStarClawRest.onTrue(new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeOpenPinch));

    JoystickButton triggerStarClawClosed = new JoystickButton(joystick0, Constants.ButtonMap.clawCone);
    triggerStarClawClosed.onTrue(new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeConePinch));

    JoystickButton triggerStarClawClosedCanRotate = new JoystickButton(joystick0, Constants.ButtonMap.poweredIntakeConePinchCanRotate);
    triggerStarClawClosedCanRotate.onTrue(new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeConePinchCanRotatePos));

    JoystickButton triggerStarClawCube = new JoystickButton(joystick0, Constants.ButtonMap.clawCube);
    triggerStarClawCube.onTrue(new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeCubePinch));

    JoystickButton triggerFlywheelIn = new JoystickButton(joystick1, Constants.ButtonMap.flywheelIn);
    triggerFlywheelIn.onTrue(new SetPoweredClawFlywheel(poweredIntake, 0.5));
    triggerFlywheelIn.onFalse(new SetPoweredClawFlywheel(poweredIntake, 0.0));
    
    JoystickButton triggerFlywheelOut = new JoystickButton(joystick1, Constants.ButtonMap.flywheelOut);
    triggerFlywheelOut.onTrue(new SetPoweredClawFlywheel(poweredIntake, -0.5));
    triggerFlywheelOut.onFalse(new SetPoweredClawFlywheel(poweredIntake, 0.0));

    JoystickButton autoConeButton = new JoystickButton(joystick0, 12);
    autoConeButton.onTrue(autoCone);

    JoystickButton stowButton = new JoystickButton(joystick0, 11);
    stowButton.onTrue(stow);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // We have to return the name of the object, which is lock for LockOnTarget in this case or the code will not work
    // We have to specify which command to run as autonomous command 
    return dock;
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
  public static Limelight getLime(){
    return lime;
  }
  public static double get_tv() {
    return 0;
  }
  public static double get_tv2() {
    return 0;
  }
  public static double get_ty() {
    return 0;
  }
  public static double get_ty2() {
    return 0;
  }

}
