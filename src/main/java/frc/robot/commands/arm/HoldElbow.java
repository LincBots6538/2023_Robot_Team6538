// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class HoldElbow extends CommandBase {
  /** Creates a new HoldElbow. */
  private final Arm sys_arm;
  private BooleanSupplier btn;
  public HoldElbow(Arm subsystem, BooleanSupplier button) {
    sys_arm = subsystem;
    btn= button;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double elbow_cmd =  sys_arm.getElbowPos();
    sys_arm.setElbow(elbow_cmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !btn.getAsBoolean();
  }
}
