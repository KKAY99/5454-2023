package frc.robot.commands;
//TODO MIGRATE HOMING
    
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PaddleSubsystem;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
 public class zHomePaddleCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  //private final PaddleSubsystem m_IntakeSubsystem;
  private final double m_speed;
  private final double m_homeTimeOut;
  private double m_StartTime=0;
  private final PaddleSubsystem m_Paddle;
 
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public zHomePaddleCommand(PaddleSubsystem paddle,double speed, double homeTimeOut) { 
    m_speed = speed;
    m_Paddle = paddle;
    m_homeTimeOut=homeTimeOut;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_StartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //disabled until we have limit switch  
  //  m_Paddle.run(m_speed); // move arms at home speed until isfinished
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Paddle.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue=false;
    double currentTime=Timer.getFPGATimestamp();
    //System.out.print("Looping " + currentTime + " "+ m_StartTime + " " + m_homeTimeOut);
    if(currentTime>m_StartTime+m_homeTimeOut){
        returnValue=true;  //Time Out period has happened
    }
    if(m_Paddle.hitPhysicalLimitSwitch()) {
        returnValue=true;  // end because hit limit switch
    }
    if(returnValue){
      m_Paddle.stop();
    
      m_Paddle.SetZero();
      m_Paddle.setHomed(true);
    }
    return returnValue;
  }
 }
