// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.c_drive;
import frc.robot.subsystems.Farfetchd;

public class drivetelliop extends CommandBase {
  /** Creates a new drivetelliop. */
  private final Farfetchd sys_Farfetchd;
  private final DoubleSupplier joystickspeed;
  private final DoubleSupplier joystickturning;
  public drivetelliop(Farfetchd in_Farfetchd, DoubleSupplier speed, DoubleSupplier turning) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_Farfetchd = in_Farfetchd;
    joystickspeed = speed;
    joystickturning = turning;
    addRequirements(sys_Farfetchd);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_Farfetchd.setPIDslot(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = MathUtil.applyDeadband(joystickspeed.getAsDouble(),c_drive.JOYSTICK_DEADBAND);
    double turning = MathUtil.applyDeadband(joystickturning.getAsDouble(),c_drive.JOYSTICK_DEADBAND);
    double sp = speed;
    double turn = Math.abs(turning) * turning;
    sys_Farfetchd.arcadedrive(sp,turn);
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
