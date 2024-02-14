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
public class armFloor extends SequentialCommandGroup {
  /** Creates a new armFloor. */
  private final Arm sys_arm;

  public armFloor(Arm subsystem){
    sys_arm = subsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    int state = sys_arm.getState();
    switch (state){
      case 0: //From Start Position
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
      case 1: // Store
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
      case 2: // Already at Floor
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
      case 3: // LVL 2
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
      case 4: // LVL 3
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
      case 5: // Shelf / High
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
      default:
        addCommands(
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1),
          new MoveXY(sys_arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2),
          new InstantCommand(sys_arm::setStateFloor)
        );
        break;
    } // end switch
  } // end function
} // end class
