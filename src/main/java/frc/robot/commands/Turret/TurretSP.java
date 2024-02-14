// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Turret;

public class TurretSP extends CommandBase {
  /** Creates a new TurretSP. */
  private final Turret sys_turret;
  private final Arm sys_arm;
  private double sp_pos;

  public TurretSP(Turret subsystem, Arm arm, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_turret = subsystem;
    sys_arm = arm;
    sp_pos = pos;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int arm_state = sys_arm.getState();
    if ((arm_state == 1) || (arm_state == 5)){
      sys_turret.SetTurretPosition(sp_pos);
    }
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
    return sys_turret.atPosition();
  }
}
