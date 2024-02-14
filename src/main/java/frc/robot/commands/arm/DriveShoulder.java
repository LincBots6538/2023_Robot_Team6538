// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DriveShoulder extends CommandBase {
  /** Creates a new DriveShoulder. */
  private final Arm sys_arm;
  private double sp_rate;
  private Timer time = new Timer();

  public DriveShoulder(Arm subsystem, double deg_s) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_arm = subsystem;
    sp_rate = deg_s / 10.0;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    sys_arm.setShoulder(sys_arm.getShoulderPos() + sp_rate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(time.get() > 0.1){
      sys_arm.setShoulder(sys_arm.getShoulderPos() + sp_rate);
      time.reset();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_arm.setShoulder(sys_arm.getShoulderPos());
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
