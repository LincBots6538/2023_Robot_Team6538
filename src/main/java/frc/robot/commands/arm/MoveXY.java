// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveXY extends CommandBase {
  /** Creates a new MoveXY. */
  private final Arm sys_arm;
  private double sp_x, sp_y;
  public MoveXY(Arm subsystem, double arm_x, double arm_y) {
    sys_arm = subsystem;
    sp_x = arm_x;
    sp_y = arm_y;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_arm.goToPosition(sp_x, sp_y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_arm.atPosition();
  }
}
