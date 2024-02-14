// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class DriveWrist extends CommandBase {
  /** Creates a new DriveWrist. */
  private final Arm sys_arm;
  private double sp_rate;
  private Timer time = new Timer();

  public DriveWrist(Arm subsystem, double pctpwr) {
    sys_arm = subsystem;
    sp_rate = pctpwr;
    // sp_rate = deg_s / 10.0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // time.start();
    // sys_arm.setWrist(sys_arm.getWristPos() + sp_rate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sys_arm.driveWrist(sp_rate);
    // if(time.get() > 0.1){
    //   sys_arm.setWrist(sys_arm.getWristPos() + sp_rate);
    //   time.reset();
    //   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_arm.driveWrist(0);
    // sys_arm.setWrist(sys_arm.getWristPos());
    // time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
