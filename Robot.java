// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

private final PWMVictorSPX leftMotor = new PWMVictorSPX(0);
private final PWMVictorSPX rightMotor = new PWMVictorSPX(1);
private final Spark lift = new Spark(3);
private final Spark intake = new Spark(2); //single motor
private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor,rightMotor); // create drive system
private final Joystick stick = new Joystick(0); // usb device channel 
private final Timer a_timer = new Timer();
private final Spark left_shooter = new Spark(4);
private final Spark right_shooter = new Spark(5);
private boolean isLiftTime;
private final AnalogPotentiometer barf = new AnalogPotentiometer(0,195,12);
private final  AnalogPotentiometer rrt = new AnalogPotentiometer(1,195,12);
private final  AnalogPotentiometer ballfinder = new AnalogPotentiometer(2,195,12);
//private final Encoder encoder = new Encoder(0,1);
//private final Encoder leftencoder = new Encoder(2,3);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture();
    rightMotor.setInverted(true);
    right_shooter.setInverted(true);
    isLiftTime = false;\

    
    
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *dr
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {  smartdashboard.putNumber("ballfinder", ballfinder.get());
 }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    a_timer.reset();
    a_timer.start(); 
  }//end of autonomousInit()

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
       
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        if (a_timer.get() <= 2.0) {
          rightMotor.set(0.3);
          leftMotor.set(0.3);
          intake.set(-0.5);
        } 
        if (a_timer.get() >= 2.3 && a_timer.get() < 6.0) {
          intake.stopMotor();
          robotDrive.arcadeDrive(0, 0.4);
          
        } 
        if (a_timer.get() >= 6.3 && a_timer.get() < 8.0) {
          right_shooter.set(-.75);
          intake.set(-0.4);
        }
        if (a_timer.get()>=8.0)
          right_shooter.stopMotor();
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotDrive.arcadeDrive(stick.getY(), stick.getX());


    if(ballfinder.get()<

    if(stick.getRawButtonReleased(1))
        isLiftTime = !isLiftTime;
        
    if(isLiftTime)
      {
        right_shooter.stopMotor();
        left_shooter.stopMotor();
        
        lift.set(stick.getRawAxis(5));
        
      }
    else
      {
        lift.stopMotor();
        //right_shooter.set(stick.getRawAxis(5));
        //left_shooter.set(stick.getRawAxis(5));
      }
intake.set(stick.getRawAxis(3) - stick.getRawAxis(2));

    if (stick.getRawButton(5)) {
      right_shooter.set(-0.75);
    }
    else if (stick.getRawButton(6)){
      right_shooter.set(-0.50);
    }
    else {
      right_shooter.set(0.0); 
    }
 
  }//end of teleopPeriodic

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
