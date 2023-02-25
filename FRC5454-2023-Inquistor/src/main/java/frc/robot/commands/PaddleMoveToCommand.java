// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.PaddleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PaddleMoveToCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final PaddleSubsystem m_paddleSubsystem;
  private final double m_speed;
  private final double m_targetPos;
  private final double m_tolerance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PaddleMoveToCommand(PaddleSubsystem paddle,double position,double tolerance, double speed) {
    m_paddleSubsystem = paddle;
    m_speed=speed;
    m_targetPos=position;
    m_tolerance=tolerance;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_paddleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_paddleSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_paddleSubsystem.checkandMoveTowardsPosition(m_targetPos, m_speed, m_tolerance);

  }
}
