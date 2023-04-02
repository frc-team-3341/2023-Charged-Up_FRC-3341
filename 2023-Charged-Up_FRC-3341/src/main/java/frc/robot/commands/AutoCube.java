// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoweredIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCube extends SequentialCommandGroup {
  /**
   * Creates a new AutoCube
   * @param dt - Drivetrain Subsystem
   * @param arm - Arm Subsystem
   * @param poweredIntake - Powered Intake Subsystem
   */
  public AutoCube(DriveTrain dt, Arm arm, PoweredIntake poweredIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    poweredIntake.setWristServoPos(0);
    poweredIntake.setClawPos(Constants.Measurements.poweredIntakeCubePinch);
    addCommands(new Rotate(arm, 95), new AutoDrive(dt, 0.2, 0, true, true), new Extend(arm, 15.65), new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeOpenPinch), new Stow(dt, arm, poweredIntake), new AutoDrive(dt, -1.0, -0.8, false, false));
  }
}
