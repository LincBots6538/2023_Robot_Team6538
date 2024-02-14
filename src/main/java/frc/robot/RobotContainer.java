// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.c_arm;
import frc.robot.commands.Turret.TurretSP;
import frc.robot.commands.arm.DriveElbow;
import frc.robot.commands.arm.DriveShoulder;
import frc.robot.commands.arm.DriveWrist;
import frc.robot.commands.arm.EnableArm;
import frc.robot.commands.arm.MoveXY;
import frc.robot.commands.arm.armFeeder;
import frc.robot.commands.arm.armFloor;
import frc.robot.commands.arm.armLVL2;
import frc.robot.commands.arm.armLVL3;
import frc.robot.commands.arm.armStore;
import frc.robot.commands.auto.LVL3auto;
import frc.robot.commands.auto.SwagAuto;
import frc.robot.commands.drive.PositionLock;
import frc.robot.commands.drive.driveTeleopTriggers;
import frc.robot.commands.grabber.DriveIntake;
import frc.robot.commands.grabber.OpenIntake;
import frc.robot.commands.grabber.SetCone;
import frc.robot.commands.grabber.ToggleGamePiece;
import frc.robot.commands.grabber.setCube;
import frc.robot.commands.grabber.setOpen;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Farfetchd;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Farfetchd my_Farfetchd = new Farfetchd();
  private Arm sys_Arm = new Arm();
  private Turret sys_Turret = new Turret();
  private Grabber sys_Grabber = new Grabber();

  private SendableChooser<Command> sel_auto = new SendableChooser<>(); 

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_manipController = new XboxController(OperatorConstants.KMANIPCONTROLLERPORT);

  //private SendableChooser<Command> db_driveChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Default Drive Command
    my_Farfetchd.setDefaultCommand(new driveTeleopTriggers(my_Farfetchd,
       m_driverController::getLeftTriggerAxis,
        m_driverController::getRightTriggerAxis,
         m_driverController::getRightX));

    // Auton chooser
    sel_auto.setDefaultOption("Swag Auto", new SwagAuto(my_Farfetchd, sys_Grabber, sys_Turret, sys_Arm));
    sel_auto.addOption("Level 3 Auto", new LVL3auto(my_Farfetchd, sys_Grabber, sys_Turret, sys_Arm));
    sel_auto.addOption("Do Nothing", null);

    Shuffleboard.getTab("auto").add(sel_auto);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Declare Controller Buttons / Triggers & Assign Commands

    //#region Manip Controller
      //#region Declare buttons
    JoystickButton c1_a = new JoystickButton(m_manipController, Button.kA.value);
    JoystickButton c1_b = new JoystickButton(m_manipController, Button.kB.value);
    JoystickButton c1_x = new JoystickButton(m_manipController, Button.kX.value);
    JoystickButton c1_y = new JoystickButton(m_manipController, Button.kY.value);
    JoystickButton c1_lb = new JoystickButton(m_manipController, Button.kLeftBumper.value);
    JoystickButton c1_rb = new JoystickButton(m_manipController, Button.kRightBumper.value);
    JoystickButton c1_ls = new JoystickButton(m_manipController, Button.kLeftStick.value);
    JoystickButton c1_rs = new JoystickButton(m_manipController, Button.kRightStick.value);
    POVButton c1_povL = new POVButton(m_manipController, 270);
    POVButton c1_povU = new POVButton(m_manipController, 0);
    POVButton c1_povR = new POVButton(m_manipController, 90);
    POVButton c1_povD = new POVButton(m_manipController, 180);
    Trigger c1_rtrig = new Trigger(this::isRightTrigger1);
    Trigger c1_ltrig = new Trigger(this::isLeftTrigger1);
      //#endregion
    Trigger c1_mode2 = c1_ltrig;
      //#region Declare Mode 1 & Mode 2 Varibles
    // Mode 1 - Normal buttons
    Trigger c1_a_mode1 = c1_a.and(c1_mode2.negate());
    Trigger c1_b_mode1 = c1_b.and(c1_mode2.negate());
    Trigger c1_x_mode1 = c1_x.and(c1_mode2.negate());
    Trigger c1_y_mode1 = c1_y.and(c1_mode2.negate());
    Trigger c1_povL_mode1 = c1_povL.and(c1_mode2.negate());
    Trigger c1_povU_mode1 = c1_povU.and(c1_mode2.negate());
    Trigger c1_povR_mode1 = c1_povR.and(c1_mode2.negate());
    Trigger c1_povD_mode1 = c1_povD.and(c1_mode2.negate());
    Trigger c1_lb_mode1 = c1_lb.and(c1_mode2.negate());
    Trigger c1_rb_mode1 = c1_rb.and(c1_mode2.negate());
    Trigger c1_rtrig_mode1 = c1_rb.and(c1_mode2.negate());

    // Mode 2 - hold left trigger
    Trigger c1_a_mode2 = c1_mode2.and(c1_a);
    Trigger c1_b_mode2 = c1_mode2.and(c1_b);
    Trigger c1_x_mode2 = c1_mode2.and(c1_x);
    Trigger c1_y_mode2 = c1_mode2.and(c1_y);
    Trigger c1_povL_mode2 = c1_mode2.and(c1_povL);
    Trigger c1_povR_mode2 = c1_mode2.and(c1_povR);
    Trigger c1_povU_mode2 = c1_mode2.and(c1_povU);
    Trigger c1_povD_mode2 = c1_mode2.and(c1_povD);
    Trigger c1_lb_mode2 = c1_mode2.and(c1_lb);
    Trigger c1_rb_mode2 = c1_mode2.and(c1_rb);
    Trigger c1_rtrig_mode2 = c1_mode2.and(c1_rtrig);
      //#endregion
    // Manip controller commands
    // Mode 1
    c1_a_mode1.toggleOnTrue(new EnableArm(sys_Arm));
    c1_b_mode1.onTrue(new MoveXY(sys_Arm, c_arm.SP_POS_FLOOR_X1, c_arm.SP_POS_FLOOR_Y1));
    c1_y_mode1.onTrue(new MoveXY(sys_Arm, c_arm.SP_POS_SHELF_X, c_arm.SP_POS_SHELF_Y));
    c1_x_mode1.onTrue(new MoveXY(sys_Arm, c_arm.SP_POS_FLOOR_X2, c_arm.SP_POS_FLOOR_Y2));
    c1_rtrig.onTrue(new setOpen(sys_Grabber));
    c1_ls.onTrue(new SetCone(sys_Grabber));
    c1_rs.onTrue(new setCube(sys_Grabber));
    c1_lb.whileTrue(new DriveIntake(sys_Grabber, .5));    // Out
    c1_rb.whileTrue(new DriveIntake(sys_Grabber, -0.5));          // In
    c1_povL_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, 90));
    c1_povU_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, 0));
    c1_povR_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, -90));
    c1_povD_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, 180));
    
    // Mode 2
    c1_a_mode2.whileTrue(new DriveShoulder(sys_Arm, -20));
    c1_b_mode2.whileTrue(new DriveShoulder(sys_Arm, 20));
    c1_x_mode2.whileTrue(new DriveElbow(sys_Arm, -20));
    c1_y_mode2.whileTrue(new DriveElbow(sys_Arm, 20));
    c1_rtrig_mode2.whileTrue(new DriveWrist(sys_Arm, .2));
    c1_rb_mode2.whileTrue(new DriveWrist(sys_Arm, -.2));
    c1_povL_mode2.whileTrue(new DriveWrist(sys_Arm, -20));
    c1_povR_mode2.whileTrue(new DriveWrist(sys_Arm, 20)); 
    c1_povD_mode2.onTrue(new MoveXY(sys_Arm, c_arm.SP_POS_STORE_X1, c_arm.SP_POS_STORE_Y1));
    c1_povU_mode2.onTrue(new MoveXY(sys_Arm, c_arm.SP_POS_HIGH_X,c_arm.SP_POS_HIGH_Y));
    //#endregion

    //#region Drive Controller
      //#region Declare buttons
    JoystickButton c0_a = new JoystickButton(m_driverController, Button.kA.value);
    JoystickButton c0_b = new JoystickButton(m_driverController, Button.kB.value);
    JoystickButton c0_x = new JoystickButton(m_driverController, Button.kX.value);
    JoystickButton c0_y = new JoystickButton(m_driverController, Button.kY.value);
    JoystickButton c0_lb = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    JoystickButton c0_rb = new JoystickButton(m_driverController, Button.kRightBumper.value);
    JoystickButton c0_ls = new JoystickButton(m_driverController, Button.kLeftStick.value);
    JoystickButton c0_rs = new JoystickButton(m_driverController, Button.kRightStick.value);
    JoystickButton c0_back = new JoystickButton(m_driverController, Button.kBack.value);
    JoystickButton c0_start = new JoystickButton(m_driverController, Button.kStart.value);
    POVButton c0_povU = new POVButton(m_driverController, 0);
    POVButton c0_povR = new POVButton(m_driverController, 90);
    POVButton c0_povD = new POVButton(m_driverController, 180);
    POVButton c0_povL = new POVButton(m_driverController, 270);
    Trigger c0_rtrig = new Trigger(this::isRightTrigger0);
    Trigger c0_ltrig = new Trigger(this::isLeftTrigger0);
    //#endregion
    Trigger c0_mode2 = c0_lb;   // Button to Hold for Mode 2
      //#region MODE 1 MODE2 Varibles
    // MODE1
    Trigger c0_a_mode1 = c0_a.and(c0_mode2.negate());
    Trigger c0_b_mode1 = c0_b.and(c0_mode2.negate());
    Trigger c0_x_mode1 = c0_x.and(c0_mode2.negate());
    Trigger c0_y_mode1 = c0_y.and(c0_mode2.negate());
    Trigger c0_rb_mode1 = c0_rb.and(c0_mode2.negate());
    Trigger c0_ls_mode1 = c0_ls.and(c0_mode2.negate());
    Trigger c0_rs_mode1 = c0_rs.and(c0_mode2.negate());
    Trigger c0_povU_mode1 = c0_povU.and(c0_mode2.negate());
    Trigger c0_povR_mode1 = c0_povR.and(c0_mode2.negate());
    Trigger c0_povD_mode1 = c0_povD.and(c0_mode2.negate());
    Trigger c0_povL_mode1 = c0_povL.and(c0_mode2.negate());
    Trigger c0_rtrig_mode1 = c0_rtrig.and(c0_mode2.negate());
    Trigger c0_ltrig_mode1 = c0_ltrig.and(c0_mode2.negate());
    // MODE 2
    Trigger c0_a_mode2 = c0_a.and(c0_mode2);
    Trigger c0_b_mode2 = c0_b.and(c0_mode2);
    Trigger c0_x_mode2 = c0_x.and(c0_mode2);
    Trigger c0_y_mode2 = c0_y.and(c0_mode2);
    Trigger c0_rb_mode2 = c0_rb.and(c0_mode2);
    Trigger c0_ls_mode2 = c0_ls.and(c0_mode2);
    Trigger c0_rs_mode2 = c0_rs.and(c0_mode2);
    Trigger c0_povU_mode2 = c0_povU.and(c0_mode2);
    Trigger c0_povR_mode2 = c0_povR.and(c0_mode2);
    Trigger c0_povD_mode2 = c0_povD.and(c0_mode2);
    Trigger c0_povL_mode2 = c0_povL.and(c0_mode2);
    Trigger c0_rtrig_mode2 = c0_rtrig.and(c0_mode2);
    Trigger c0_ltrig_mode2 = c0_ltrig.and(c0_mode2);
    //#endregion

    // Default Drive Command uses left / right triggers & right stick X axis
    c0_lb.whileTrue(new PositionLock(my_Farfetchd));
    //c0_rb_mode1.whileTrue(new OpenIntake(sys_Grabber));
    c0_rb_mode1.whileTrue(new DriveIntake(sys_Grabber, -.75));
    c0_rb_mode2.whileTrue(new DriveIntake(sys_Grabber, .75));
    c0_povL_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, 90));
    c0_povU_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, 0));
    c0_povR_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, -90));
    c0_povD_mode1.onTrue(new TurretSP(sys_Turret, sys_Arm, 180));
    c0_back.toggleOnTrue(new EnableArm(sys_Arm));
    c0_start.onTrue(new ToggleGamePiece(sys_Grabber));
    c0_a_mode1.onTrue(new armFloor(sys_Arm));
    c0_b_mode1.onTrue(new armLVL2(sys_Arm, sys_Grabber));
    c0_y_mode1.onTrue(new armLVL3(sys_Arm, sys_Grabber));
    c0_x_mode1.onTrue(new armStore(sys_Arm));

    c0_a_mode2.onTrue(new armFeeder(sys_Arm));


    //#endregion
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    //return new SimpleAuto(my_Farfetchd, sys_Grabber);
    //return new SwagAuto(my_Farfetchd, sys_Grabber, sys_Turret, sys_Arm);
    //return new driveForward(my_Farfetchd, 5);
    // Level 3 auto
    //return new LVL3auto(my_Farfetchd, sys_Grabber, sys_Turret, sys_Arm);
    return sel_auto.getSelected();

  }


  //#region Controller Booleans
  public boolean isRightTrigger1(){
    return m_manipController.getRightTriggerAxis() > 0.3;
  }
  public boolean isLeftTrigger1(){
    return m_manipController.getLeftTriggerAxis() > 0.3;
  }
  public boolean isRightTrigger0(){
    return m_driverController.getRightTriggerAxis() > 0.3;
  }
  public boolean isLeftTrigger0(){
    return m_driverController.getLeftTriggerAxis() > 0.3;
  }
  //#endregion
}
