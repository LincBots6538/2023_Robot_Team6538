// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleGamePiece extends InstantCommand {
  private final Grabber sys_grabber;
  public ToggleGamePiece(Grabber subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_grabber = subsystem;
    addRequirements(sys_grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(sys_grabber.getState() == 1) {
      sys_grabber.setCube();
    } else if(sys_grabber.getState() == 2){
      sys_grabber.setCone();
    } else if(sys_grabber.getState() == 0) {
      sys_grabber.setCube();
    }
  }
}
