// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeConveySubsystem;
import frc.robot.subsystems.PaddleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmsSubsystem;

/** An example command that uses an example subsystem. */
public class PaddleConveyRetractCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final PaddleSubsystem m_PaddleSubsystem;
  private final IntakeConveySubsystem m_ConveySubsystem;
  private final IntakeArmsSubsystem m_IntakeArmsSubsystem;
  private final double m_paddleSpeed;
  private final double m_conveySpeed;
  private final double m_armsSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PaddleConveyRetractCommand(PaddleSubsystem paddle, IntakeArmsSubsystem intakeArmsSubsystem, IntakeConveySubsystem convey, double paddleSpeed,double ConveySpeed, double armsSpeed) {
    m_PaddleSubsystem = paddle;
    m_ConveySubsystem = convey;
    m_paddleSpeed=paddleSpeed;
    m_conveySpeed=ConveySpeed;
    m_armsSpeed = armsSpeed;
    m_IntakeArmsSubsystem = intakeArmsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_PaddleSubsystem);
    addRequirements(m_ConveySubsystem);
    addRequirements(m_IntakeArmsSubsystem);
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
    m_IntakeArmsSubsystem.run(m_armsSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_PaddleSubsystem.stop();
    m_ConveySubsystem.stop();
    m_IntakeArmsSubsystem.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
