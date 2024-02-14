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
public class armStore extends SequentialCommandGroup {
  /** Creates a new armStore. */
  private final Arm sys_arm;
  public armStore(Arm subsystem) {
    sys_arm = subsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    int state = sys_arm.getState();
    switch (state){
      case 0: //From Start Position
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
          new InstantCommand(sys_arm::setStateHome)
        );
        break;
      case 1: // Already at store -> Move to Shelf
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
          new InstantCommand(sys_arm::setStateHome)
        );
        break;
      case 2:
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
          new InstantCommand(sys_arm::setStateHome)
        );
        break;
      case 3:
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
          new InstantCommand(sys_arm::setStateHome)
        );
        break;
      case 4:
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
          new InstantCommand(sys_arm::setStateHome)
        );
        break;
      case 5:
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_STORE_X2, c_arm.SP_POS_STORE_Y2),
          new InstantCommand(sys_arm::setStateHome)
        );
        break;


    } 
  }
}

