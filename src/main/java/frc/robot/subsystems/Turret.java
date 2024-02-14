// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.c_Turret;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  // Turret Motor
  private TalonFX m_Turret = new TalonFX(c_Turret.MotorID);
  
  // Turret position variables
  private double turret_pos_cmd, turret_pos_act, tpc_raw, tpa_raw;

  public Turret() {
    // Set initial encoder position 
    m_Turret.setSelectedSensorPosition(0);  
    turret_pos_act = 0;
    turret_pos_cmd = 0;
    
    m_Turret.setNeutralMode(NeutralMode.Brake);

    m_Turret.setSensorPhase(true);

    m_Turret.configClosedloopRamp(0.5);
    
    m_Turret.config_kF(0, c_Turret.kF_value);
    m_Turret.config_kP(0, c_Turret.kP_value);
    m_Turret.config_kI(0, c_Turret.kI_value);
    m_Turret.config_kD(0, c_Turret.kD_value);
    m_Turret.configAllowableClosedloopError(0, c_Turret.PID_error);

    m_Turret.configClosedLoopPeakOutput(0, c_Turret.MaxOutput);

    m_Turret.selectProfileSlot(0, 0);

    
  }

  public void SetTurretPosition(double deg){
    // Function to set the desired Turret position
    turret_pos_cmd = deg;                           // Store the requested postion in degrees
    tpc_raw = turret_pos_cmd*c_Turret.tickRate;     // Store the commanded position in sensor units
  }

  public double getTurretPosition(){
    return m_Turret.getSelectedSensorPosition() / c_Turret.tickRate;
  }

  public boolean atPosition(){
    return Math.abs(turret_pos_act-turret_pos_cmd) < .5;
  }

  private void updateDashboard(){
    SmartDashboard.putNumber("Turret Position", turret_pos_act);
    SmartDashboard.putNumber("Commanded Turret Position", turret_pos_cmd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tpa_raw = m_Turret.getSelectedSensorPosition(); // Get the current turret position in sensor units
    turret_pos_act = tpa_raw / c_Turret.tickRate;   // Convert the current position to degrees
    updateDashboard();

    // Set the turret motor setpoint
    m_Turret.set(TalonFXControlMode.Position, tpc_raw);

  }
}
