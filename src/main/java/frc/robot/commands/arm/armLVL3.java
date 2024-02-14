// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.c_arm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class armLVL3 extends SequentialCommandGroup {
  /** Creates a new armLVL3. */
  private final Arm sys_arm;
  private final Grabber sys_Grabber;

  public armLVL3(Arm sub_arm, Grabber sub_grab) {
    sys_arm = sub_arm;
    sys_Grabber = sub_grab;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
        new MoveXY(sys_arm, c_arm.SP_POS_LVL3CUBE_X, c_arm.SP_POS_LVL3CUBE_Y),
        new InstantCommand(sys_arm::setStateLvl3)
      );
    
    // if (sys_Grabber.getState() == 1){
    //   addCommands(
    //     new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
    //     new MoveXY(sys_arm, c_arm.SP_POS_LVL3CONE_X, c_arm.SP_POS_LVL3CONE_Y),
    //     new InstantCommand(sys_arm::setStateLvl3)
    //   );
    // }
    // else {
    //   addCommands(
    //     new MoveXY(sys_arm, c_arm.SP_POS_HIGH_X, c_arm.SP_POS_HIGH_Y),
    //     new MoveXY(sys_arm, c_arm.SP_POS_LVL3CUBE_X, c_arm.SP_POS_LVL3CUBE_Y),
    //     new InstantCommand(sys_arm::setStateLvl3)
    //   );
    // }
    // sys_arm.setState(4);
  }
}
