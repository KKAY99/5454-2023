// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeConveySubsystem;
import frc.robot.subsystems.PaddleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeArmsSubsystem;
import frc.robot.Constants.HungryHippoValues;

/** An example command that uses an example subsystem. */
public class zHungryHippoCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final PaddleSubsystem m_PaddleSubsystem;
  private final IntakeConveySubsystem m_ConveySubsystem;
  private final IntakeArmsSubsystem m_IntakeArmsSubsystem;
  
  private enum STATE {

    EXTENDOUT,ROTATEANDEXTEND,DROPPADDLE,ROTATEANDRETRACTP1,ROTATEANDRETRACTP2,
    ROTATEANDRETRACTP3,ROTATEANDRETRACTP4,ROTATEANDRETRACTP5,FINISHRETRACT,ABORT,END
  }
  private STATE m_state;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zHungryHippoCommand(Integer limitSwitch, PaddleSubsystem paddle, IntakeArmsSubsystem intakeArmsSubsystem, IntakeConveySubsystem convey) {
    m_PaddleSubsystem = paddle;
    m_ConveySubsystem = convey;
    m_IntakeArmsSubsystem = intakeArmsSubsystem;
    m_state=STATE.EXTENDOUT;
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
    boolean returnValue=false; // default is we are not finished unless we abort or end
    switch(m_state){
        case EXTENDOUT:
        if(m_IntakeArmsSubsystem.getPos() <HungryHippoValues.armPos1){
            m_IntakeArmsSubsystem.runwithLimits(HungryHippoValues.armSpeed1);
        } else{
            m_IntakeArmsSubsystem.stop();
            m_state=STATE.END;
        }
      
        break;
        case ROTATEANDEXTEND:
            boolean armExtend=false;
            boolean paddleRotated=false;
            if (m_IntakeArmsSubsystem.getPos() <HungryHippoValues.armPos2){
                m_IntakeArmsSubsystem.runwithLimits(HungryHippoValues.armSpeed2);
            } else {
                m_IntakeArmsSubsystem.stop();
            }
            if (m_PaddleSubsystem.getPos() <HungryHippoValues.paddlePos2){
                m_PaddleSubsystem.run(HungryHippoValues.paddleSpeed2);
            }else {
                m_PaddleSubsystem.stop();
            }
            if(armExtend && paddleRotated){
                m_state=STATE.END;
            }

        break;
        case DROPPADDLE:
        break;
        case ROTATEANDRETRACTP1:
        break;
        case ROTATEANDRETRACTP2:
        break;
        case ROTATEANDRETRACTP3:
        break;
        case ROTATEANDRETRACTP4:
        break;
        case ROTATEANDRETRACTP5:
        break;
        case FINISHRETRACT:
        break;
        case ABORT:
        case END:
        returnValue=true;
        break;    

    }    
    return returnValue;
   // m_PaddleSubsystem.run(m_paddleSpeed);
    //m_ConveySubsystem.run(m_conveySpeed);
    //m_IntakeArmsSubsystem.run(m_armsSpeed);
}
}
