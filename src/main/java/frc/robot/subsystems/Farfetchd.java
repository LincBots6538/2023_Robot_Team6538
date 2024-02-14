// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.c_drive;

public class Farfetchd extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX sc_frontright_wheely = new TalonFX(c_drive.MotorID_FR);
  private TalonFX sc_frontleft_wheely = new TalonFX(c_drive.MotorID_FL);
  private TalonFX sc_backright_wheely = new TalonFX(c_drive.MotorID_RR);
  private TalonFX sc_backleft_wheely = new TalonFX(c_drive.MotorID_RL);

  private double rdrive_cmd, ldrive_cmd, rpos_cmd, lpos_cmd;
  
  public Farfetchd (){
    // Set Gearbox Kinematics
    sc_backright_wheely.follow(sc_frontright_wheely);
    sc_backleft_wheely.follow(sc_frontleft_wheely);

    // Set controllers to coast
    sc_backleft_wheely.setNeutralMode(NeutralMode.Coast);
    sc_backright_wheely.setNeutralMode(NeutralMode.Coast);
    sc_frontleft_wheely.setNeutralMode(NeutralMode.Coast);
    sc_frontright_wheely.setNeutralMode(NeutralMode.Coast);

    // Set motor phase
    sc_frontleft_wheely.setSensorPhase(true);
    sc_frontright_wheely.setSensorPhase(true);

    // Ramp Rates
    sc_frontleft_wheely.configClosedloopRamp(c_drive.rampRate);
    sc_frontright_wheely.configClosedloopRamp(c_drive.rampRate);

    // Slot 0 PID values (velocity control)
    sc_frontleft_wheely.config_kP(0, c_drive.kP_value_0);
    sc_frontleft_wheely.config_kI(0, c_drive.kI_value_0);
    sc_frontleft_wheely.config_kD(0, c_drive.kD_value_0);
    sc_frontleft_wheely.config_kF(0, c_drive.kF_value_0);
    sc_frontleft_wheely.configAllowableClosedloopError(0, c_drive.PID_error_0);
    sc_frontleft_wheely.configClosedLoopPeakOutput(0, c_drive.MaxOutput_0);

    sc_frontright_wheely.config_kP(0, c_drive.kP_value_0);
    sc_frontright_wheely.config_kI(0, c_drive.kI_value_0);
    sc_frontright_wheely.config_kD(0, c_drive.kD_value_0);
    sc_frontright_wheely.config_kF(0, c_drive.kF_value_0);    
    sc_frontright_wheely.configAllowableClosedloopError(0, c_drive.PID_error_0);    
    sc_frontright_wheely.configClosedLoopPeakOutput(0, c_drive.MaxOutput_0);

    // Slot 1 PID values (Position control - turning)
    sc_frontleft_wheely.config_kP(1, c_drive.kP_value_1);
    sc_frontleft_wheely.config_kI(1, c_drive.kI_value_1);
    sc_frontleft_wheely.config_kD(1, c_drive.kD_value_1);
    sc_frontleft_wheely.config_kF(1, c_drive.kF_value_1);
    sc_frontleft_wheely.configAllowableClosedloopError(1, c_drive.PID_error_1);
    sc_frontleft_wheely.configClosedLoopPeakOutput(1, c_drive.MaxOutput_1);
    
    sc_frontright_wheely.config_kP(1, c_drive.kP_value_1);    
    sc_frontright_wheely.config_kI(1, c_drive.kI_value_1);    
    sc_frontright_wheely.config_kD(1, c_drive.kD_value_1);    
    sc_frontright_wheely.config_kF(1, c_drive.kF_value_1);
    sc_frontright_wheely.configAllowableClosedloopError(1, c_drive.PID_error_1);    
    sc_frontright_wheely.configClosedLoopPeakOutput(1, c_drive.MaxOutput_1);

    // Slot 2 PID values (Position control - straight)
    sc_frontleft_wheely.config_kP(2, c_drive.kP_value_2);
    sc_frontleft_wheely.config_kI(2, c_drive.kI_value_2);
    sc_frontleft_wheely.config_kD(2, c_drive.kD_value_2);
    sc_frontleft_wheely.config_kF(2, c_drive.kF_value_2);
    sc_frontleft_wheely.configAllowableClosedloopError(2, c_drive.PID_error_2);
    sc_frontleft_wheely.configClosedLoopPeakOutput(2, c_drive.MaxOutput_2);
    
    sc_frontright_wheely.config_kP(2, c_drive.kP_value_2);    
    sc_frontright_wheely.config_kI(2, c_drive.kI_value_2);    
    sc_frontright_wheely.config_kD(2, c_drive.kD_value_2);    
    sc_frontright_wheely.config_kF(2, c_drive.kF_value_2);
    sc_frontright_wheely.configAllowableClosedloopError(2, c_drive.PID_error_2);    
    sc_frontright_wheely.configClosedLoopPeakOutput(2, c_drive.MaxOutput_2);

  }

  public void setPIDslot(int slot){
    // This changes the PID slot to use
    sc_frontleft_wheely.selectProfileSlot(slot, 0);
    sc_frontright_wheely.selectProfileSlot(slot, 0);
  }

  public void arcadedrive(double speed,double turn){
    //System.out.println("speed " + speed + ", turn " + turn);
    rdrive_cmd = speed + (.5*turn);
    ldrive_cmd = -(speed - (.5*turn));

    sc_frontright_wheely.set(ControlMode.Velocity, rdrive_cmd * c_drive.Max_Velocity_raw);
    sc_frontleft_wheely.set(ControlMode.Velocity, ldrive_cmd * c_drive.Max_Velocity_raw);
  }

  /**
   * Set the position set point for drive motors
   * @param right positive values are forward
   * @param left  negative values are forward
   */
  public void positionDrive(double right, double left){
    rpos_cmd = right;
    lpos_cmd = left;
    sc_frontleft_wheely.set(ControlMode.Position, left);
    sc_frontright_wheely.set(ControlMode.Position, right);
  }

  /**
   * Return the position value of the drivetrain.
   * @param left true returns the left side value, false returns the right
   * @return encoder value
   */
  public double getPosition(boolean left){
    if(left == true){
      return sc_frontleft_wheely.getSelectedSensorPosition();
    }
    else{
      return sc_frontright_wheely.getSelectedSensorPosition();
    }
  }

  public boolean atSetPoint(){
    //return (sc_frontleft_wheely.getClosedLoopError() < c_drive.PID_error_0) && (sc_frontright_wheely.getClosedLoopError() < c_drive.PID_error_0);
    return (lpos_cmd-sc_frontleft_wheely.getSelectedSensorPosition() < c_drive.PID_error_2) && (rpos_cmd-sc_frontright_wheely.getSelectedSensorPosition() < c_drive.PID_error_2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
