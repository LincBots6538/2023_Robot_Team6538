// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.c_drive;
import frc.robot.subsystems.Farfetchd;

public class driveForward extends CommandBase {
  /** Creates a new driveForward. */
  private final Farfetchd sys_drive;
  private double sp_dis, sp_l, sp_r;


  /**
   * 
   * @param subsystem drive subsystem
   * @param dis Distance in feet
   */
  public driveForward(Farfetchd subsystem, double dis) {
    sys_drive = subsystem;
    sp_dis = dis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_drive.setPIDslot(2);

    double distance = Math.round(((c_drive.Ratio*Constants.FX_encoder_cnt)*(1.0/((c_drive.Wheel_Dia/12.0) * Math.PI)))* sp_dis);

    sp_l = sys_drive.getPosition(true)+distance;
    sp_r = sys_drive.getPosition(false)-distance;


    sys_drive.positionDrive(sp_r, sp_l);

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
  //   if(Math.abs(sys_drive.getPosition(false))> 0.9*Math.abs(sp_l)){
  //   return true;
  // }
  // else{
  //   return false;
  // }
  return sys_drive.atSetPoint();
  }
}
