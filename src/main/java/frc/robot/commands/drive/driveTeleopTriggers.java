// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.c_drive;
import frc.robot.subsystems.Farfetchd;

public class driveTeleopTriggers extends CommandBase {
  /** Creates a new driveTeleopTriggers. */
  private final Farfetchd sys_drive;
  private DoubleSupplier fwd, rev, turn;
  private double speedAxis, speedVal, turnVal;
  public driveTeleopTriggers(Farfetchd subsystem, DoubleSupplier fwdspeed, DoubleSupplier revspeed, DoubleSupplier turning) {
    sys_drive = subsystem;
    fwd = fwdspeed;
    rev = revspeed;
    turn = turning;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sys_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_drive.setPIDslot(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speedAxis = fwd.getAsDouble() - rev.getAsDouble();
    speedVal = MathUtil.applyDeadband(speedAxis, c_drive.JOYSTICK_DEADBAND);
    turnVal = MathUtil.applyDeadband(turn.getAsDouble(), c_drive.JOYSTICK_DEADBAND);
    double sp = Math.abs(speedVal) * speedVal;
    double tr = Math.abs(turnVal) * turnVal;
    sys_drive.arcadedrive(sp, tr);
    
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
