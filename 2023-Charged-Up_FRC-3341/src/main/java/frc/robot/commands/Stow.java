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
public class Stow extends SequentialCommandGroup {
  /** Creates a new BlueLeft. */
  public Stow(DriveTrain dt, Arm arm, PoweredIntake poweredIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeOpenPinch), new Extend(arm, 3), new AutoDrive(dt, -0.5, 0, true, true), new Rotate(arm, 15));

    //addCommands(new SetPoweredClawPos(poweredIntake, Constants.Measurements.poweredIntakeOpenPinch), new SetWristPosPI(poweredIntake, 0), new Extend(arm, 3), new AutoDrive(dt, -0.5, 0, true, true), new Rotate(arm, 15));
  }
}
