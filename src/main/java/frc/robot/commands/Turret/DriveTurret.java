// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class DriveTurret extends CommandBase {
  /** Creates a new DriveTurret. */
  private final Turret sys_turret;
  private double sp_spd, sp_inc;
  private Timer time = new Timer();


  public DriveTurret(Turret subsystem, double deg_s) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_turret = subsystem;
    sp_inc = deg_s / 10.0;
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
    sp_spd = sys_turret.getTurretPosition()+sp_inc;
    sys_turret.SetTurretPosition(sp_spd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((time.get() > .1))
    sp_spd = sys_turret.getTurretPosition() + sp_inc;
    sys_turret.SetTurretPosition(sp_spd);
    time.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_turret.SetTurretPosition(sys_turret.getTurretPosition());
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
