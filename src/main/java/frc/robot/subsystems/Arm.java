// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.c_arm;

public class Arm extends SubsystemBase {
  /** Controls  for arm. */
  // sc_ mean speed controelr// 
  private TalonFX sc_shouldermotor = new TalonFX(c_arm.kshouldermotorCANID);
  private TalonSRX sc_elbowmotor = new TalonSRX(c_arm.kelbowmotorCANID);
  private TalonSRX sc_wristmotor = new TalonSRX(c_arm.kwristmotorCANID);
  
  private double shoulder_cmd_raw, shoulder_cmd, elbow_cmd_raw, elbow_cmd, wrist_cmd_raw, wrist_cmd;
  private double shoulder_act_raw, shoulder_act, elbow_act_raw, elbow_act, wrist_act_raw, wrist_act;
  private int armState = 5;
  private boolean PIDactive = false;
  private Timer t_sensor = new Timer();
  

  public Arm() {
    // Set up Shoulder Motor
    sc_shouldermotor.configFactoryDefault();
    sc_shouldermotor.setNeutralMode(NeutralMode.Brake);
    sc_shouldermotor.setInverted(TalonFXInvertType.CounterClockwise);
    //sc_shouldermotor.configPeakOutputForward(c_arm.MaxOutput_shd);
    //sc_shouldermotor.configPeakOutputReverse(c_arm.MaxOutput_shd);
    sc_shouldermotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    sc_shouldermotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    StatorCurrentLimitConfiguration curlim_shd = new StatorCurrentLimitConfiguration();
    curlim_shd.currentLimit = 50;
    curlim_shd.enable = true;
    sc_shouldermotor.configStatorCurrentLimit(curlim_shd);
    
    
    sc_shouldermotor.config_kP(0, c_arm.kP_value_shd);
    sc_shouldermotor.config_kF(0, c_arm.kF_value_shd);
    sc_shouldermotor.config_kI(0, c_arm.kI_value_shd);
    sc_shouldermotor.config_kD(0, c_arm.kD_value_shd);
    sc_shouldermotor.configAllowableClosedloopError(0, c_arm.PID_error_shd);
    sc_shouldermotor.configClosedLoopPeakOutput(0, c_arm.MaxOutput_shd);

    sc_shouldermotor.selectProfileSlot(0, 0);
    shoulder_cmd = c_arm.start_pos_shd;
    shoulder_cmd_raw = Math.round(shoulder_cmd * c_arm.TickRate_shd);
    sc_shouldermotor.setSelectedSensorPosition(shoulder_cmd_raw);

    sc_shouldermotor.configMotionCruiseVelocity(c_arm.cruise_speed_shd);
    sc_shouldermotor.configMotionAcceleration(c_arm.max_speed_shd);
    sc_shouldermotor.configMotionSCurveStrength(0);

    // Set up Elbow Motor
    sc_elbowmotor.configFactoryDefault();
    sc_elbowmotor.setNeutralMode(NeutralMode.Brake);
    sc_elbowmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    sc_elbowmotor.setSensorPhase(true); 
    sc_elbowmotor.setInverted(false);
    //sc_elbowmotor.configPeakOutputForward(c_arm.MaxOutput_elb);
    //sc_elbowmotor.configPeakOutputReverse(c_arm.MaxOutput_down_elb); // limit output driving down

    sc_elbowmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    sc_elbowmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    // PID settings
    sc_elbowmotor.config_kP(0, c_arm.kP_value_elb);
    sc_elbowmotor.config_kF(0, c_arm.kF_value_elb);
    sc_elbowmotor.config_kI(0, c_arm.kI_value_elb);
    sc_elbowmotor.config_kD(0, c_arm.kD_value_elb);
    sc_elbowmotor.configAllowableClosedloopError(0, c_arm.PID_error_elb);
    sc_elbowmotor.configClosedLoopPeakOutput(0, c_arm.MaxOutput_elb);
    
    // Motion Magic settings
    sc_elbowmotor.configMotionCruiseVelocity(c_arm.cruise_speed_elb);
    sc_elbowmotor.configMotionAcceleration(c_arm.cruise_speed_elb * 3);
    sc_elbowmotor.configMotionSCurveStrength(0);
    

    sc_elbowmotor.selectProfileSlot(0, 0);
    elbow_cmd = c_arm.start_pos_elb;
    elbow_cmd_raw = Math.round(elbow_cmd * c_arm.TickRate_elb);
    sc_elbowmotor.setSelectedSensorPosition(-elbow_cmd_raw);  // sensor phase can apply negative

    sc_elbowmotor.configPeakCurrentLimit(0);
    sc_elbowmotor.configContinuousCurrentLimit(40);
    

    // Set up Wrist Motor
    sc_wristmotor.configFactoryDefault();
    sc_wristmotor.setNeutralMode(NeutralMode.Brake);
    sc_wristmotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    sc_wristmotor.setSensorPhase(true);
    sc_wristmotor.setInverted(true);

    sc_wristmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    sc_wristmotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    sc_wristmotor.config_kP(0, c_arm.kP_value_wrs);
    sc_wristmotor.config_kF(0, c_arm.kF_value_wrs);
    sc_wristmotor.config_kI(0, c_arm.kI_value_wrs);
    sc_wristmotor.config_kD(0, c_arm.kD_value_wrs);
    sc_wristmotor.configAllowableClosedloopError(0, c_arm.PID_error_wrs);
    sc_wristmotor.configClosedLoopPeakOutput(0, c_arm.MaxOutput_wrs);
    
    // Motion Magic settings
    sc_wristmotor.configMotionCruiseVelocity(c_arm.cruise_speed_wrs);
    sc_wristmotor.configMotionAcceleration(c_arm.cruise_speed_wrs * 3);
    sc_wristmotor.configMotionSCurveStrength(0);

    sc_wristmotor.selectProfileSlot(0, 0);
    wrist_cmd = c_arm.start_pos_wrs;
    wrist_cmd_raw = Math.round(wrist_cmd * c_arm.TickRate_wrs);
    sc_wristmotor.setSelectedSensorPosition(wrist_cmd_raw);

    sc_wristmotor.configPeakCurrentLimit(0);
    sc_wristmotor.configContinuousCurrentLimit(10);
    sc_wristmotor.enableCurrentLimit(true);
    
    t_sensor.start();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
    if(wrist_cmd > 220){
      wrist_cmd = 220;
      wrist_cmd_raw = wrist_cmd*c_arm.TickRate_wrs; 
    }
    else if (wrist_cmd < 80){
      wrist_cmd = 80;
      wrist_cmd_raw = wrist_cmd*c_arm.TickRate_wrs; 
    }

    if (PIDactive){
      sc_elbowmotor.set(ControlMode.MotionMagic, elbow_cmd_raw);
      sc_shouldermotor.set(ControlMode.MotionMagic, shoulder_cmd_raw);
      sc_wristmotor.set(ControlMode.MotionMagic, wrist_cmd_raw);
    }

    SmartDashboard.putBoolean("Arm Enabled?", PIDactive);

    shoulder_act_raw = sc_shouldermotor.getSelectedSensorPosition();
    shoulder_act = shoulder_act_raw / c_arm.TickRate_shd;
    //if (shoulder_act < 85)  shoulder_act = shoulder_act - c_arm.GEAR_LASH;
    SmartDashboard.putNumber("Shoulder Pos", shoulder_act);
    SmartDashboard.putNumber("Shoulder Cmd", shoulder_cmd);

    elbow_act_raw = sc_elbowmotor.getSelectedSensorPosition();
    elbow_act = elbow_act_raw / c_arm.TickRate_elb;
    SmartDashboard.putNumber("Elbow Pos", elbow_act);
    SmartDashboard.putNumber("Elbow Cmd", elbow_cmd);

    wrist_act_raw = sc_wristmotor.getSelectedSensorPosition();
    wrist_act = wrist_act_raw / c_arm.TickRate_wrs;
    SmartDashboard.putNumber("Wrist Pos", wrist_act);
    SmartDashboard.putNumber("Wrist Cmd", wrist_cmd);

    SmartDashboard.putBoolean("Arm at Position", atPosition());
    SmartDashboard.putNumber("arm state", armState);

    // set sensor positons 200 ms after boot-up
    if (t_sensor.get() > .2){
      sc_wristmotor.setSelectedSensorPosition(wrist_cmd_raw);
      sc_elbowmotor.setSelectedSensorPosition(elbow_cmd_raw);
      sc_shouldermotor.setSelectedSensorPosition(shoulder_cmd_raw);
      armState = 1;
      t_sensor.reset();
      t_sensor.stop();
    }
  }

  public boolean atPosition(){
    if ((Math.abs(shoulder_act-shoulder_cmd ) < 1) && 
        (Math.abs(elbow_act-elbow_cmd ) < 1)){
      return true;
    }
    else return false;
  }

  //Sets the motors to the correct positions such that the grabber is at the x,y coordinates specified
  public void goToPosition(double x, double y) 
  {
      // Validate x,y command postions (i.e. too low /high, too far, etc)
    
      double v = Math.sqrt(x*x + y*y);  //the hypotenuse
      double gamma = Math.asin(y/v);    //angle between robot base and desired x,y coordinates

      double L1 = Constants.c_arm.arm1_length_in;   //length of first arm segment
      double L2 = Constants.c_arm.arm2_length_in;   //length of second arm segment

      double phi = Math.acos((L1*L1 + L2*L2 - v*v)/(2*L1*L2)); //angle of elbow motor (between arms)

      double theta = Math.asin((L2/v)*Math.sin(phi)) + gamma; //angle of shoulder motor

      double omega = theta + phi;   //wrist motor angle

      shoulder_cmd = Math.toDegrees(theta);
      elbow_cmd = Math.toDegrees(phi);
      wrist_cmd = Math.toDegrees(omega);

      // Account for Gear Lash
      if (shoulder_cmd < 85)  shoulder_cmd = shoulder_cmd + c_arm.GEAR_LASH;
      
      // convert to sensor units
      shoulder_cmd_raw = Math.round(shoulder_cmd * c_arm.TickRate_shd);
      elbow_cmd_raw = Math.round(elbow_cmd * c_arm.TickRate_elb);
      wrist_cmd_raw = Math.round(wrist_cmd * c_arm.TickRate_wrs);

  }
public void setShoulder(double pos){
  shoulder_cmd = pos;
  shoulder_cmd_raw = Math.round(shoulder_cmd * c_arm.TickRate_shd);   
}

public void setElbow(double pos){
  elbow_cmd = pos;
  elbow_cmd_raw = Math.round(elbow_cmd * c_arm.TickRate_elb);
}

public void setWrist(double pos){
  wrist_cmd = pos;
  wrist_cmd_raw = Math.round(wrist_cmd * c_arm.TickRate_wrs);
}

public void driveWrist(double pwr){
  sc_wristmotor.set(ControlMode.PercentOutput, pwr);
}

/**
 * Set the positionn of the Arm
 * @param ArmPos - 0: Start 1: Store 2: Floor 3: Lvl2 4: lvl3 5: shelf
 */
public void setState(int ArmPos){
  armState = ArmPos;
  SmartDashboard.putNumber("arm state", armState);
}

public void setStateHome(){
  armState = 1;
}

public void setStateFloor(){
  armState = 2;
}

public void setStateLvl2(){
  armState = 3;
}

public void setStateLvl3(){
  armState = 4;
}

public void setStateHigh(){
  armState = 5;
}

/**
 * Get the positionn of the Arm
 * Returns - 0: Start 1: Store 2: Floor 3: Lvl2 4: lvl3 5: shelf
 */
public int getState(){
  return armState;
}

public void enable(boolean state){
  PIDactive = state;

  // When the PID control is enabled, set the setpoints to the current positions. act varibles refreshed in the periodic loop
  if (state){
    shoulder_cmd_raw = shoulder_act_raw;
    elbow_cmd_raw = elbow_act_raw;
    wrist_cmd_raw = wrist_act_raw;

    shoulder_cmd = shoulder_cmd_raw / c_arm.TickRate_shd;
    elbow_cmd = elbow_cmd_raw / c_arm.TickRate_elb;
    wrist_cmd = wrist_cmd_raw / c_arm.TickRate_wrs; 
  }
  else{
    sc_elbowmotor.set(ControlMode.PercentOutput,0);
    sc_shouldermotor.set(ControlMode.PercentOutput,0);
    sc_wristmotor.set(ControlMode.PercentOutput,0);
  }
}

public void enableTrue(){

  shoulder_cmd_raw = shoulder_act_raw;
  elbow_cmd_raw = elbow_act_raw;
  wrist_cmd_raw = wrist_act_raw;

  shoulder_cmd = shoulder_cmd_raw / c_arm.TickRate_shd;
  elbow_cmd = elbow_cmd_raw / c_arm.TickRate_elb;
  wrist_cmd = wrist_cmd_raw / c_arm.TickRate_wrs; 
}

/**
 * Returns Elbow position
 * @return Angle between UPPER ARM and LOWER ARM in degrees
 */
public double getElbowPos(){
  return elbow_act;
}

/**
 * Returns Should Position
 * @return Angle between TURRET PLATE and LOWER ARM in degrees
 */
public double getShoulderPos(){
  return shoulder_act;
}

/**
 * Returns Wrist Position
 * @return Angle between the top of the GRABBER and top of the UPPER ARM in degrees
 */
public double getWristPos(){
  return wrist_act;
}
}