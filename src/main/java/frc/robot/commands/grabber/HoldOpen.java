// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class HoldOpen extends CommandBase {
  /** Creates a new HoldOpen. */
  private final Grabber sys_grabber;
  private int state;
  public HoldOpen(Grabber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_grabber = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = sys_grabber.getState();
    sys_grabber.setOpen();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(state == 1) sys_grabber.setCone();
    if(state == 2) sys_grabber.setCube();
    if(state == 0) sys_grabber.setCube();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
