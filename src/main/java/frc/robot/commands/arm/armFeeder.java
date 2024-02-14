// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.c_arm;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class armFeeder extends SequentialCommandGroup {
  private final Arm sys_arm;
  /** Creates a new armFeeder. */
  public armFeeder(Arm sub_arm) {
    sys_arm = sub_arm;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
        new MoveXY(sys_arm, 12, 24),
        new InstantCommand(sys_arm::setStateHome)
    );
  }
}
