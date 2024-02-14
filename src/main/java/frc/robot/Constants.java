// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int FX_encoder_cnt = 2048;
  public static final int SRX_encoder_cnt = 4096; 


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int KMANIPCONTROLLERPORT = 1;
  } 

  public static class c_drive{
    public static final int MotorID_FR = 2;
    public static final int MotorID_RR = 3;
    public static final int MotorID_FL = 0;
    public static final int MotorID_RL = 1;

    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double rampRate = 1;
    public static final double Max_Velocity = 12.0;
    public static final double Ratio = 10.75;
    public static final double Wheel_Dia = 6.0; 
    public static final double Max_Velocity_raw = Math.round(Max_Velocity * (1/(Math.PI*(Wheel_Dia/12))) * (FX_encoder_cnt*Ratio)* (1.0/10.0)); 
    //public static final double Max_Velocity_raw = Max_Velocity * (1/(3.1416*(Wheel_Dia/12))) * (FX_encoder_cnt*Ratio)* (1.0/10.0); 
    
    public static final double WIDTH = 22.0;

    public static final double MaxOutput_0 = 1.0;
    public static final double PID_error_0 = 100;
    public static final double kP_value_0 = 0.005;
    public static final double kF_value_0 = 0.05;
    public static final double kI_value_0 = 0;
    public static final double kD_value_0 = 0;

    public static final double MaxOutput_1 = 0.5;
    public static final double PID_error_1 = 25;
    public static final double kP_value_1 = 0.05;
    public static final double kF_value_1 = 0;
    public static final double kI_value_1 = 0;
    public static final double kD_value_1 = 0;

    public static final double MaxOutput_2 = .5;
    public static final double PID_error_2 = 1000;
    public static final double kP_value_2 = 0.02;
    public static final double kF_value_2 = 0;
    public static final double kI_value_2 = 0;
    public static final double kD_value_2 = 0;
  }

  // c_ for constants 
  public static class c_arm{

    public static final int kshouldermotorCANID = 13;
    public static final int kelbowmotorCANID = 5;
    public static final int kwristmotorCANID = 9; 

    public static final double arm1_length_in = 34.0;
    public static final double arm2_length_in = 32.5;

    public static final double SP_POS_START_X = 3.16;
    public static final double SP_POS_START_Y = 2.75;

    public static final double SP_POS_STORE_X1 = 7.3;
    public static final double SP_POS_STORE_Y1 = 5.0;
    public static final double SP_POS_STORE_X2 = 5.0;
    public static final double SP_POS_STORE_Y2 = 3.5;

    public static final double SP_POS_FLOOR_X1 = 20.0;
    public static final double SP_POS_FLOOR_Y1 = 10.0;
    public static final double SP_POS_FLOOR_X2 = 20.0;
    public static final double SP_POS_FLOOR_Y2 = -1; //-7.75

    public static final double SP_POS_HIGH_X = 22;  //22
    public static final double SP_POS_HIGH_Y = 25;  //25

    public static final double SP_POS_SHELF_X = 17;    //  14 + 3.5 + 14 - 11
    public static final double SP_POS_SHELF_Y = 31.5;  //  38 + 4.5 - 11.25

    public static final double SP_POS_LVL2CUBE_X = 20.5;  // 14 + 14 - 11         17
    public static final double SP_POS_LVL2CUBE_Y = 23;  // 24 + 6 - 11.25       18.75

    public static final double SP_POS_LVL2CONE_X = 33.5;  // 22 + 14 - 6          30
    public static final double SP_POS_LVL2CONE_Y = 26.25;  // 34 + 5 -1.5 - 11.25  26.25

    public static final double SP_POS_LVL3CUBE_X = 38.5;  // 32 + 14 - 11         35
    public static final double SP_POS_LVL3CUBE_Y = 42;  // 36 + 6 - 11.25       30.75

    public static final double SP_POS_LVL3CONE_X = 51.5;  // 40 + 14 - 6          48
    public static final double SP_POS_LVL3CONE_Y = 38.25;  // 46 + 5 - 1.5 - 11.25 38.25
    
    public static final double start_pos_shd = 112;   // angle between the front turret plate and lower arm in degrees
    public static final double ratio_shd = (48.0/16.0)*64.0*(36.0/18.0);
    public static final double TickRate_shd = FX_encoder_cnt*ratio_shd/360.0;
    public static final double GEAR_LASH = 7.0; // gear lash in deg
    public static final double GEAR_LASH_TICKS = GEAR_LASH * TickRate_shd;
    public static final double max_speed_shd = (6380.0 / ratio_shd)  * (360.0 / 60.0) * TickRate_shd / 10.0;  // ticks/100ms
    public static final double cruise_speed_shd = 50.0 * TickRate_shd / 10; // ticks/100ms - Set w/ first term in deg/s
    public static final double MaxOutput_shd = 0.5;
    public static final double MaxOutput_down_shd = 0.25;
    public static final double PID_error_shd = 100;
    public static final double kP_value_shd = 0.1;
    public static final double kF_value_shd = 0;
    public static final double kI_value_shd = 0;
    public static final double kD_value_shd = 0;
    
    public static final double start_pos_elb = 6.5;   // angle between lower and  upper arms in degrees 
    public static final double ratio_elb = (36.0/18.0);
    public static final double TickRate_elb = SRX_encoder_cnt*ratio_elb/360.0;
    public static final double max_speed_elb = (18700.0 / 600.0)  * (360.0 / 60.0) * TickRate_elb / 10.0;  // ticks/100ms
    public static final double cruise_speed_elb = 90.0 * TickRate_elb / 10; // ticks/100ms - Set w/ first term in deg/s
    public static final double MaxOutput_elb = 1;
    public static final double MaxOutput_down_elb = .5;
    public static final double PID_error_elb = 15;
    public static final double kP_value_elb = 10;  // 1.76% output per 1 deg of error
    public static final double kF_value_elb = 0;
    public static final double kI_value_elb = 0;
    public static final double kD_value_elb = 0;
    
    public static final double start_pos_wrs = 119;   // angle between the top plate of grabber and upper arm in degrees
    public static final double ratio_wrs = (22.0/16.0);
    public static final double TickRate_wrs = SRX_encoder_cnt*ratio_wrs/360.0;
    public static final double max_speed_wrs = (18700.0 / 412.5)  * (360.0 / 60.0) * TickRate_wrs / 10.0;  // ticks/100ms
    public static final double cruise_speed_wrs = 90.0 * TickRate_wrs / 10; // ticks/100ms - Set w/ first term in deg/s
    public static final double MaxOutput_wrs = 0.75;
    public static final double MaxOutput_down_wrs = -0.25;
    public static final double PID_error_wrs = 10;
    public static final double kP_value_wrs = 1;
    public static final double kF_value_wrs = 0;
    public static final double kI_value_wrs = 0;
    public static final double kD_value_wrs = 0;
    
    
  }

  public static class c_Turret{

    public static final int MotorID = 12;
    public static final double ratio = (60.0/16.0)* (250.0/14.0); 
    public static final double MaxOutput = .25;
    public static final double tickRate = FX_encoder_cnt*ratio/360.0;   // ticks per deg of turret rotation
    
    public static final double PID_error = 1000;
    public static final double kP_value = 0.1;
    public static final double kF_value = 0;
    public static final double kI_value = 0;
    public static final double kD_value = 0;

  }

public static class c_Grabber{
  public static final int MOTORID_RIGHT = 11;
  public static final int MOTORID_LEFT = 4;

  public static final int PWMID_RIGHT = 0;
  public static final int PWMID_LEFT = 1;

  public static final double SP_OPEN = 0;
  public static final double SP_CUBE = .3;
  public static final double SP_CONE = .5;
}


}

