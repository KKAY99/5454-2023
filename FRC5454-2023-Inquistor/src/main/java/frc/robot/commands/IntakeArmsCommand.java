// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.PaddleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmsSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeArmsCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  //private final PaddleSubsystem m_IntakeSubsystem;
  private final double m_speed;
  private final IntakeArmsSubsystem m_intakeArms;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public IntakeArmsCommand(IntakeArmsSubsystem intakeArms,double speed) { 
    m_speed = speed;
    m_intakeArms = intakeArms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //  m_intakeArms.runwithLimits(m_speed);
    m_intakeArms.runNOEncoders(m_speed);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeArms.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return  m_intakeArms.atLimit(m_speed); // If Intake Arms are at the limit then will return trun
  
  }
}
