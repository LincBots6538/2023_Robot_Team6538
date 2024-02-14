// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.c_arm;
import frc.robot.commands.Turret.TurretSP;
import frc.robot.commands.arm.EnableArm;
import frc.robot.commands.arm.MoveXY;
import frc.robot.commands.drive.driveForward;
import frc.robot.commands.grabber.setIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Farfetchd;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LVL3auto extends SequentialCommandGroup {
  /** Creates a new LVL3auto. */
  public LVL3auto(Farfetchd sys_drive, Grabber sys_grab, Turret sys_turret, Arm sys_arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
      new EnableArm(sys_arm).repeatedly(),
      new SequentialCommandGroup(
        new WaitCommand(.3),
        new TurretSP(sys_turret, sys_arm, 180),
        new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
        new MoveXY(sys_arm, c_arm.SP_POS_LVL3CUBE_X, c_arm.SP_POS_LVL3CUBE_Y),
        new setIntake(sys_grab, .5),
        new WaitCommand(.5),
        new setIntake(sys_grab, 0),
        new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
        new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
        new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
        new TurretSP(sys_turret, sys_arm, 0),
        new driveForward(sys_drive, 12)
        )
      )
    );
  }
}
