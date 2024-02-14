// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.c_Grabber;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private TalonSRX sc_rightIntake = new TalonSRX(c_Grabber.MOTORID_RIGHT);
  private TalonSRX sc_leftIntake = new TalonSRX(c_Grabber.MOTORID_LEFT);

  private Servo svo_leftarm = new Servo(0);
  private Servo svo_rightarm = new Servo(1);

  private int gr_state =1;

  public Grabber() {
    sc_rightIntake.follow(sc_leftIntake);
    sc_rightIntake.setInverted(InvertType.OpposeMaster);

    svo_leftarm.setBounds(2, 1.5, 1.5, 1.5, 1);
    svo_rightarm.setBounds(2, 1.5, 1.5, 1.5, 1);
    svo_leftarm.enableDeadbandElimination(true);
    svo_rightarm.enableDeadbandElimination(true);

    sc_leftIntake.configPeakCurrentLimit(0);
    sc_leftIntake.configContinuousCurrentLimit(5);
    sc_leftIntake.enableCurrentLimit(true);

    sc_rightIntake.configPeakCurrentLimit(0);
    sc_rightIntake.configContinuousCurrentLimit(5);
    sc_rightIntake.enableCurrentLimit(true);

    setCube();
  }
  public void setOpen(){
    svo_leftarm.setPosition(c_Grabber.SP_OPEN);
    svo_rightarm.setPosition(c_Grabber.SP_OPEN);
    gr_state = 0;
  }
  public void setCube(){
    svo_leftarm.setPosition(c_Grabber.SP_CUBE);
    svo_rightarm.setPosition(c_Grabber.SP_CUBE);
    gr_state = 2;
  }
  public void setCone(){
    svo_leftarm.setPosition(c_Grabber.SP_CONE);
    svo_rightarm.setPosition(c_Grabber.SP_CONE);
    gr_state = 1;
  }

  public void setIntake(double pctpwr){
    sc_leftIntake.set(ControlMode.PercentOutput, pctpwr);
  }

  /**
   * Returns the state of Intake arms
   * @return Grip State: 0 - open/not set, 1 - cone, 2 - cube
   */
  public int getState(){
    return gr_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
