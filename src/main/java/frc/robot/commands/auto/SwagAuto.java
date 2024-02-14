// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Turret.TurretSP;
import frc.robot.commands.drive.driveForward;
import frc.robot.commands.grabber.setIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Farfetchd;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwagAuto extends SequentialCommandGroup {
  /** Creates a new SwagAuto. */
  public SwagAuto(Farfetchd sys_drive, Grabber sys_grab, Turret sys_turret, Arm sys_arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurretSP(sys_turret, sys_arm, 180),
      new setIntake(sys_grab, .5),
      new WaitCommand(1),
      new setIntake(sys_grab, 0),
      new ParallelCommandGroup(
        new driveForward(sys_drive, 12),
        new TurretSP(sys_turret, sys_arm, 0)
      )
      
    );
  }
}
