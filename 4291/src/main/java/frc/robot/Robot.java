// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String vCustomAuto = "Double Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double lrf_distance, rrf_distance, ballfinder_distance;




  private final PWMVictorSPX leftMotor = new PWMVictorSPX(0);
  private final PWMVictorSPX rightMotor = new PWMVictorSPX(1);
  private final Spark lift = new Spark(3);
  private final Spark intake = new Spark(2); //single motor
  //private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor,rightMotor);
  private final Joystick stick = new Joystick(0); // usb device channel 
  private final Timer a_timer = new Timer();
  private final Spark left_shooter = new Spark(4);
  private final Spark right_shooter = new Spark(5);
  private boolean isLiftTime;
  private boolean isautolift;
  private final AnalogInput lrf = new AnalogInput(3);
  private final  AnalogInput rrf = new AnalogInput(2);
  private final  AnalogInput ballfinder = new AnalogInput(1);
  //private final Timer intakeTimer = new Timer();
  private boolean ball;
  private final Encoder rightencoder = new Encoder(2,3);
  private final Encoder leftencoder = new Encoder(0,1);
  private double r_dist,l_dist,r_rate,l_rate;
  private double rfDifference;
  private boolean inRangeR;
  private boolean inRangeL;
  private PowerDistribution  PDP = new PowerDistribution(0, ModuleType.kCTRE);

  private WPI_VictorSPX leftFront = new WPI_VictorSPX(1);
  private WPI_VictorSPX leftFollower = new WPI_VictorSPX(2);
  private WPI_VictorSPX rightFront = new WPI_VictorSPX(3);
  private WPI_VictorSPX rightFollower = new WPI_VictorSPX(4);
  

  private DifferentialDrive robotDrive = new DifferentialDrive(leftFront, rightFront);

  private Faults faults_L = new Faults();
  private Faults faults_R = new Faults();

  private CANSparkMax shooter = new CANSparkMax(6, MotorType.kBrushed);
  private CANSparkMax turret = new CANSparkMax(5, MotorType.kBrushed);





  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //CAN drive system
    rightFront.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftFollower.configFactoryDefault();

    rightFollower.follow(rightFront);
    leftFollower.follow(leftFront);

    rightFront.setInverted(true);
    leftFront.setInverted(false);

    rightFollower.setInverted(InvertType.FollowMaster);
    leftFollower.setInverted(InvertType.FollowMaster);

    rightFront.setSensorPhase(true);
    leftFront.setSensorPhase(true);
    //end CAN drive system

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Double Auto", vCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture();
    rightMotor.setInverted(true);
    right_shooter.setInverted(true);
    isLiftTime = false;
    rightencoder.setDistancePerPulse(Math.PI * (6.0/360.0));
    leftencoder.setDistancePerPulse(Math.PI * (6.0/360.0));
    rightencoder.setReverseDirection(false);
    rightencoder.reset();
    leftencoder.reset();
    inRangeR = false;
    inRangeL = false;
    
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *dr
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {  
    if(rrf_distance <= 165 && rrf_distance >= 170) {
      inRangeR = true;
    } else {
      inRangeR = false;
    }
    if(lrf_distance <= 165 && lrf_distance >= 170) {
      inRangeL = true;
    } else {
      inRangeL = false;
    }

    lrf_distance = lrf.getVoltage() / 0.00977;
    rrf_distance = rrf.getVoltage() / 0.00977;
    ballfinder_distance = ballfinder.getVoltage() / 0.00977;
    SmartDashboard.putNumber("ballfinder", ballfinder_distance);
    SmartDashboard.putNumber("rrf", rrf_distance);
    SmartDashboard.putNumber("lrf", lrf_distance);

    r_dist=rightencoder.getDistance();
    l_dist=leftencoder.getDistance();
    r_rate = rightencoder.getRate();
    l_rate = leftencoder.getRate();
    SmartDashboard.putNumber("rightencoder",r_dist);
    SmartDashboard.putNumber("leftencoder",l_dist);
    SmartDashboard.putNumber("r_rate",r_rate);
    SmartDashboard.putNumber("l_rate",l_rate);
    SmartDashboard.putNumber("Range Finder Difference:", rfDifference);
    SmartDashboard.putBoolean("In Range Right", inRangeR);
    SmartDashboard.putBoolean("In Range Left", inRangeL);
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
    rightencoder.reset();
    leftencoder.reset();
    
  }//end of autonomousInit()

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {

      case kCustomAuto:
        // Put custom auto code here
        if(a_timer.get()<5)
        {
          right_shooter.set(-0.4);
        }
        if(a_timer.get()>5 && a_timer.get()<7)
        {
          intake.set(-.8);
        }
        if (a_timer.get()>7 && a_timer.get()<15)
        {
          intake.stopMotor();
          right_shooter.stopMotor();
          rightMotor.set(-.25);
          leftMotor.set(-.25);
        }

        break;

        
      case vCustomAuto:
      {
        if (a_timer.get()<=5)
        {
          rightMotor.set(0.25);
          leftMotor.set(0.25);
        }
        else if (a_timer.get()>5 && a_timer.get() <=7)
        {
          rightMotor.set(-.25);
          leftMotor.set(0.25);
          right_shooter.set(0.45);
        }
        else if (a_timer.get()>7 && a_timer.get()<=8)
        {
          rightMotor.set(0);
          leftMotor.set(0);
          if (rrf_distance - lrf_distance <0)
          {
            rightMotor.set(0.15);
            leftMotor.set(0);
          } 
          else if (rrf_distance - lrf_distance >0 ) 
          {
            leftMotor.set(0.15);
            rightMotor.set(0);
          } 
          else
          {
            leftMotor.set(0);
            rightMotor.set(0);
          }
          if (rrf_distance >= 182.88)
            {
              rightMotor.set(0.25);
            }
            else
            {
              rightMotor.set(0);
            }
            if (lrf_distance >= 182.88)
            {
              leftMotor.set(0.25);
            }
            else
            {
              leftMotor.set(0);
            }
        }
        else if (a_timer.get()>7 && a_timer.get()<= 9)
        {
          intake.set(-0.5);
        }
        else if (a_timer.get()>9 && a_timer.get()<=10)
        {
          intake.stopMotor();
        }
        else if (a_timer.get()>10 && a_timer.get()<=12)
        {
          intake.set(-0.5);
        }
        else if (a_timer.get()>12 && a_timer.get()<=15)
        {
          intake.stopMotor();
          right_shooter.stopMotor();
        }
        }
        
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        if (a_timer.get() <= 5) {
          if(rrf_distance >= 180){
            rightMotor.stopMotor();
          }
            if(rrf_distance <=180){
              rightMotor.set(-0.18);
            }
          if(lrf_distance >=180){
            leftMotor.stopMotor();
          }
            if(lrf_distance <=180){
              leftMotor.set(-0.18);
            }

          
        } 
        if(a_timer.get() > 5 && a_timer.get() < 11){
          right_shooter.set(-0.78);
        }
        if (a_timer.get() > 7 && a_timer.get() < 11) {
        intake.set(-0.6);
                      
        } 
        if (a_timer.get() >= 11 && a_timer.get() < 15) {
          intake.stopMotor();
          right_shooter.stopMotor();
        }
        if(a_timer.get()>=12 && a_timer.get()<15){
          rightMotor.set(-0.2);
          leftMotor.set(-0.2);
        }
       


        break;
        
    }
  }



  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
   
    rightencoder.reset();
    leftencoder.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // robotDrive.arcadeDrive(stick.getY() * -0.8, stick.getX()*0.8);

   //CAN Robot Drive
   double forward = -1*stick.getRawAxis(1);
   double turn = +1*stick.getRawAxis(0);

   if (Math.abs(forward) < 0.1)
        forward = 0;
   if (Math.abs(turn) < 0.1)
        turn = 0;
   
   robotDrive.arcadeDrive(forward, turn);
  //End CAN Robot Drive

  
  /*if(stick.getRawButtonReleased(2))
     isautolift =!isautolift;
     
     {
      if(ballfinder_distance < 35.000 || ballfinder_distance > 39){ 
      ball = true;}
      else{
        intake.set(stick.getRawAxis(3) - stick.getRawAxis(2));
      }
    }*/
    
    if(stick.getRawButtonReleased(2))
    isautolift = !isautolift;{
      if(isautolift)
  {
    if(ballfinder_distance < 30.0){ 
      intake.set(-0.5);}
    else{
      intake.set(stick.getRawAxis(3) - stick.getRawAxis(2));
    }
      
    
    
  }
else
  {
    intake.set(stick.getRawAxis(3) - stick.getRawAxis(2));
  }
    }

    

    

    
    

  
       
      


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
      if(ball) {
        intake.set(-0.5);
      } else { 
      
      }

    if (stick.getRawButton(5)) {
      shooter.set(0.75);
    }
    else if (stick.getRawButton(6)){
      shooter.set(0.45);
    }
    else {
      shooter.stopMotor();
    }

    if(stick.getRawButton(3)){
      turret.set(0.35);
    }
    else if(stick.getRawButton(4)){
      turret.set(-0.35);
    }
    else{
      turret.stopMotor();
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