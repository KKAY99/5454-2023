// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeArmsSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeArmsMoveToCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeArmsSubsystem m_intakeArmsSubsystem;

  private final double m_targetPos;
  private final double m_speed;
  private final double m_tolerance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeArmsMoveToCommand(IntakeArmsSubsystem intake,double position,double tolerance, double speed) {
    m_intakeArmsSubsystem = intake;
    m_targetPos=position;
    m_speed=speed;
    m_tolerance=tolerance;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeArmsSubsystem);
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
    m_intakeArmsSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean returnValue = false;
    double currentPos=m_intakeArmsSubsystem.getPos();
    double distancefromTarget=m_targetPos-currentPos;
    double moveSpeed=0;
    //check if position is within tolerance and then stop command
    if(Math.abs(distancefromTarget)<=m_tolerance){
      m_intakeArmsSubsystem.stop();
      returnValue=true;
    } else{
        //outside tolerance  / Encoder gets more negative as we extend out
        // set speed to negative if we need to extend and positive if we need to retact
        if(distancefromTarget>0){
          //go negative to extend
          moveSpeed=0-Math.abs(m_speed);
        } else{
          //go positive to retract
          moveSpeed=Math.abs(m_speed);
        }

        m_intakeArmsSubsystem.runwithLimits(moveSpeed);

      }  
    return returnValue;
  }
}
