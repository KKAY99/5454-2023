// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final DoubleSupplier m_speed;
  private final Double m_maxValue;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCommand(ElevatorSubsystem elevator,DoubleSupplier speedSupplier,double maxValue) {
    m_ElevatorSubsystem  = elevator;
    m_speed=speedSupplier;
    m_maxValue=maxValue;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed=m_speed.getAsDouble();
    if(speed>m_maxValue){
      speed=m_maxValue;
    }else{
      if(speed<0-m_maxValue){
        speed=0-m_maxValue;
        }
    }
  //  speed=Math.max(speed,m_maxValue);
  //  speed=Math.min(speed,-m_maxValue);
 
    m_ElevatorSubsystem.runWithOutLimit(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ElevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
