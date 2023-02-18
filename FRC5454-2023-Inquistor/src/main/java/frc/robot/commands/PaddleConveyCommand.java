// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeConveySubsystem;
import frc.robot.subsystems.PaddleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class PaddleConveyCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final PaddleSubsystem m_PaddleSubsystem;
  private final double m_paddleSpeed;
  private final double m_conveySpeed;
  private final IntakeConveySubsystem m_ConveySubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PaddleConveyCommand(PaddleSubsystem paddle, IntakeConveySubsystem convey, double paddleSpeed,double ConveySpeed) {
    m_PaddleSubsystem = paddle;
    m_ConveySubsystem = convey;
    m_paddleSpeed=paddleSpeed;
    m_conveySpeed=ConveySpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_PaddleSubsystem);
    addRequirements(m_ConveySubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PaddleSubsystem.run(m_paddleSpeed);
    m_ConveySubsystem.run(m_conveySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PaddleSubsystem.stop();
    m_ConveySubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
