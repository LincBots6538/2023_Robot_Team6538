// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.c_drive;
import frc.robot.subsystems.Farfetchd;

public class FaceAngle extends CommandBase {
  /** Creates a new FaceAngle. */
  private final Farfetchd sys_drive;
  private double sp_angle, sp_l, sp_r;

  public FaceAngle(Farfetchd subsystem, double deg) {
    sys_drive = subsystem;
    sp_angle = deg;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance = (c_drive.WIDTH/c_drive.Wheel_Dia) * (sp_angle/360) * c_drive.Ratio * Constants.FX_encoder_cnt;

    sp_l = sys_drive.getPosition(true) + distance;
    sp_r = sys_drive.getPosition(false) + distance;

    sys_drive.setPIDslot(1);
    sys_drive.positionDrive(sp_r, sp_l);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_drive.atSetPoint();
  }
}
