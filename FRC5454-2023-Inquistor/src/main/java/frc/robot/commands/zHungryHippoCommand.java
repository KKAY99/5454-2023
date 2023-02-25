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
  public zHungryHippoCommand(PaddleSubsystem paddle, IntakeArmsSubsystem intakeArmsSubsystem, IntakeConveySubsystem convey) {
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
    boolean areArmsAtPos = false;
    boolean isPaddleAtPos = false;
    switch(m_state){
        case EXTENDOUT:
        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos1, HungryHippoValues.armSpeed1, HungryHippoValues.armTolerance)){
            m_state=STATE.END;
            //m_state=STATE.ROTATEANDEXTEND;
        }
        break;
        case ROTATEANDEXTEND:
        areArmsAtPos = false;
        isPaddleAtPos = false;
        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos2, HungryHippoValues.armSpeed2, HungryHippoValues.armTolerance)){
            areArmsAtPos=true;
        }

        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos2, HungryHippoValues.paddleSpeed2, HungryHippoValues.paddleTolerance)){
            isPaddleAtPos = true;
        }

        if(isPaddleAtPos&&areArmsAtPos){
            m_state=STATE.DROPPADDLE;
        }
        break;
        case DROPPADDLE:
        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos3, HungryHippoValues.paddleSpeed3, HungryHippoValues.paddleTolerance)){
           m_state=STATE.ROTATEANDRETRACTP1;
        }

        break;
        case ROTATEANDRETRACTP1:
        areArmsAtPos = false;
        isPaddleAtPos = false;

        m_ConveySubsystem.run(HungryHippoValues.conveySpeed);

        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos4, HungryHippoValues.armSpeed4, HungryHippoValues.armTolerance)){
            areArmsAtPos=true;
        }

        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos4, HungryHippoValues.paddleSpeed4, HungryHippoValues.paddleTolerance)){
            isPaddleAtPos = true;
        }

        if(isPaddleAtPos&&areArmsAtPos){
            m_state=STATE.ROTATEANDRETRACTP2;
        }
        break;
        case ROTATEANDRETRACTP2:
        areArmsAtPos = false;
        isPaddleAtPos = false;
        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos5, HungryHippoValues.armSpeed5, HungryHippoValues.armTolerance)){
            areArmsAtPos=true;
        }

        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos5, HungryHippoValues.paddleSpeed5, HungryHippoValues.paddleTolerance)){
            isPaddleAtPos = true;
        }

        if(isPaddleAtPos&&areArmsAtPos){
            m_state=STATE.ROTATEANDRETRACTP3;
        }
        break;
        case ROTATEANDRETRACTP3:
        areArmsAtPos = false;
        isPaddleAtPos = false;
        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos6, HungryHippoValues.armSpeed6, HungryHippoValues.armTolerance)){
            areArmsAtPos=true;
        }

        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos6, HungryHippoValues.paddleSpeed6, HungryHippoValues.paddleTolerance)){
            isPaddleAtPos = true;
        }

        if(isPaddleAtPos&&areArmsAtPos){
            m_state=STATE.ROTATEANDRETRACTP4;
        }
        break;
        case ROTATEANDRETRACTP4:
        areArmsAtPos = false;
        isPaddleAtPos = false;
        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos7, HungryHippoValues.armSpeed7, HungryHippoValues.armTolerance)){
            areArmsAtPos=true;
        }

        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos7, HungryHippoValues.paddleSpeed7, HungryHippoValues.paddleTolerance)){
            isPaddleAtPos = true;
        }

        if(isPaddleAtPos&&areArmsAtPos){
            m_state=STATE.ROTATEANDRETRACTP5;
        }
        break;
        case ROTATEANDRETRACTP5:
        areArmsAtPos = false;
        isPaddleAtPos = false;
        if(m_IntakeArmsSubsystem.checkandMoveTowardsPosition(HungryHippoValues.armPos8, HungryHippoValues.armSpeed8, HungryHippoValues.armTolerance)){
            areArmsAtPos=true;
        }

        if(m_PaddleSubsystem.checkandMoveTowardsPosition(HungryHippoValues.paddlePos8, HungryHippoValues.paddleSpeed8, HungryHippoValues.paddleTolerance)){
            isPaddleAtPos = true;
        }

        if(isPaddleAtPos&&areArmsAtPos){
            m_state=STATE.FINISHRETRACT;
        }
        break;
        case FINISHRETRACT:
        m_IntakeArmsSubsystem.stop();
        m_PaddleSubsystem.stop();
        m_ConveySubsystem.stop();
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
