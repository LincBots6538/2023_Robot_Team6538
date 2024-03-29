// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class OpenIntake extends CommandBase {
  /** Creates a new OpenIntake. */
  private final Grabber sys_grabber;
  public OpenIntake(Grabber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_grabber = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_grabber.setOpen();
    sys_grabber.setIntake(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_grabber.setCube();
    sys_grabber.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
