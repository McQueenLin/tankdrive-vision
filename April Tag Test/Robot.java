// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.Arrays;
import java.util.Collections;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private XboxController mController = new XboxController(0);
  private static final int leftFrontDeviceID = 4;
  private static final int leftBackDeviceID = 3;
  private static final int rightFrontDeviceID = 1;
  private static final int rightBackDeviceID = 2;

  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_leftBackMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_rightBackMotor;
  private Command m_autonomousCommand;


  private RobotContainer m_robotContainer;

  PhotonCamera camera = new PhotonCamera("team4");
  DifferentialDrive m_drive;

  final double ANGULAR_P = 0.03;
  final double ANGULAR_D = 0.003;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.3
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_leftFrontMotor = new CANSparkMax(4, MotorType.kBrushless);
    m_leftBackMotor = new CANSparkMax(3, MotorType.kBrushless);
    m_rightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_rightBackMotor = new CANSparkMax(2, MotorType.kBrushless);
    MotorControllerGroup leftMotors = new MotorControllerGroup(m_leftBackMotor, m_leftFrontMotor);
    MotorControllerGroup rightMotors = new MotorControllerGroup(m_rightBackMotor, m_rightFrontMotor);
    rightMotors.setInverted(true);
  

    m_drive = new DifferentialDrive(leftMotors, rightMotors);
    

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    var result = camera.getLatestResult();
    
    double rotation = 0;
    double speed = 0;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  
    if(result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      double y = Photonvison.getRotation();
      Transform3d ddd = target.getBestCameraToTarget();
      double x = Photonvison.getDistance()*100;
      SmartDashboard.putNumber("Y", y);
      if(x != 0){
        if(x < 40){
          speed = -0.3;
        } else if(x < 90){
          speed = -0.2;
        } else if(x > 150){
          speed = 0.3;
        } else if(x > 110){
          speed = 0.2;
        } else{
          speed = 0;
        }
      } 
    }
    
      
    
    
    
     
    m_drive.arcadeDrive(speed, rotation);
    SmartDashboard.putNumber("Speed", speed);
    
    //  if(y > 200 && y< 450){
    //     rotation = 0.3;
    //   } else if(y < 170 && y > 0){
    //     rotation = -0.3; 
    //   } else{
    //     rotation = mController.getRightX();
    //   }
    // } else{
    //   speed = mController.getLeftY()*0.3;
    //   rotation = mController.getRightX()*0.5;
     
    

  
}
  


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
