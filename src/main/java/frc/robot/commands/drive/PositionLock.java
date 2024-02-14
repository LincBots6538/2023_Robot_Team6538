// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Farfetchd;

public class PositionLock extends CommandBase {
  /** Creates a new PositionLock. */
  private final Farfetchd sys_drive;
  

  private double rtarget, ltarget;

  public PositionLock(Farfetchd subsystem) {
    sys_drive = subsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_drive.setPIDslot(1);
    rtarget = sys_drive.getPosition(false);
    ltarget = sys_drive.getPosition(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sys_drive.positionDrive(rtarget, ltarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
