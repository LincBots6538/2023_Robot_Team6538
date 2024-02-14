// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DriveElbow extends CommandBase {
  /** Creates a new DriveElbow. */
  private final Arm sys_arm;
  private double sp_rate;
  private Timer time = new Timer();

  public DriveElbow(Arm subsystem, double rate) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_arm = subsystem;
    sp_rate = rate / 10;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    sys_arm.setElbow(sys_arm.getElbowPos() + sp_rate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() > 0.1){
    sys_arm.setElbow(sys_arm.getElbowPos() + sp_rate);
    time.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_arm.setElbow(sys_arm.getElbowPos());
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
