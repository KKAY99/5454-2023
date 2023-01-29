// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.net.PortForwarder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoModes; 
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.io.File;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.LogFileUtil;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SendableChooser<Integer> m_autoChooser = new SendableChooser<Integer>();
  private SendableChooser<Integer> m_delayChooser = new SendableChooser<Integer>();
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

  if (isReal()) {
      String logPath="C:" + File.separator + "RobotLogs" + File.separator;
      Logger.getInstance().addDataReceiver(new WPILOGWriter(logPath)); // Log to a USB stick
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
  } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
  }

  Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    m_robotContainer = new RobotContainer();
    
 
    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
    
    m_autoChooser.addOption(AutoModes.autoMode0, AutoModes.autoNothing);
    m_autoChooser.addOption(AutoModes.autoMode1, AutoModes.autoMoveForward);
    m_autoChooser.addOption(AutoModes.autoMode2, AutoModes.autoCubeLeave);
    m_autoChooser.addOption(AutoModes.autoMode3, AutoModes.autoConeLeave);
    m_autoChooser.addOption(AutoModes.autoMode4, AutoModes.autoCubeDock);
    m_autoChooser.addOption(AutoModes.autoMode5, AutoModes.autoConeDock);
    m_autoChooser.addOption(AutoModes.autoMode6, AutoModes.autoCubeEngage);
    m_autoChooser.addOption(AutoModes.autoMode7, AutoModes.autoConeEngage);
    m_autoChooser.setDefaultOption(AutoModes.autoMode6, AutoModes.autoCubeEngage);
    m_delayChooser.addOption(AutoModes.delayMode0,AutoModes.delayValMode0);
    m_delayChooser.addOption(AutoModes.delayMode1,AutoModes.delayValMode1);
    m_delayChooser.addOption(AutoModes.delayMode2,AutoModes.delayValMode2);
    m_delayChooser.addOption(AutoModes.delayMode3,AutoModes.delayValMode3);
    
    SmartDashboard.putData("Auto Selector", m_autoChooser);
    SmartDashboard.putData("Delay Time", m_delayChooser);
    CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //KK - Update dashboard
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.resetDriveModes();
    m_robotContainer.DisableMode();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disableLimelights();
    m_robotContainer.disabledPerioidicUpdates();
    m_robotContainer.refreshSmartDashboard();
    
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.EnableMode();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
  }
    m_robotContainer.enableLimelights();
    m_robotContainer.resetDriveModes();
    m_robotContainer.LEDAutoMode();
    //m_robotContainer.resetTurret(); move to command groups
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoChooser.getSelected());
    
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.enableLimelights();
    m_robotContainer.resetDriveModes();
    m_robotContainer.TeleopMode();
    m_robotContainer.EnableMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.refreshSmartDashboard();
  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
