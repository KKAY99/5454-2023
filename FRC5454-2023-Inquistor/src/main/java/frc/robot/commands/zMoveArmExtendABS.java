package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Constants.Lift;
import frc.robot.Constants.TargetHeight;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class zMoveArmExtendABS extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final ElevatorSubsystem m_elevator;
  private final RotateArmSubsystem m_rotate;
  private double m_posInitLift;
  private double m_posFullLiftStage1;
  private double m_posFullLiftStage2;
  private double m_angleStage1ABS;
  private Constants.TargetHeight m_targetLevel;
  private double m_angleStage2ABS;
  private static int kTolerance= 5;
  private static enum STATE
  {
                  INITLIFT,ROTATE,EXTENDANDROTATE,EXTENDLIFT,FINALROTATE,ABORT,END
  }
 private STATE m_state=STATE.INITLIFT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmExtendABS(ElevatorSubsystem elevator, RotateArmSubsystem rotate, Constants.TargetHeight targetLevel) {
    m_elevator = elevator;
    m_rotate = rotate;
    m_targetLevel=targetLevel;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
    switch(targetLevel){
      case TOP:
      m_posFullLiftStage1=Constants.Lift.posHighFullLiftStage1;
      m_posFullLiftStage2=Constants.Lift.posHighFullLiftStage2;
      m_angleStage1ABS=Constants.Rotate.angleHighConeStage1ABS;
      m_angleStage2ABS=Constants.Rotate.angleHighConeStage2ABS;
      break;
      case MIDDLE:
        m_posFullLiftStage1=Constants.Lift.posMiddleFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posMiddleFullLiftStage2;
        m_angleStage1ABS=Constants.Rotate.angleMiddleConeStage1ABS;
        m_angleStage2ABS=Constants.Rotate.angleMiddleConeStage2ABS;
      break;
      case BOTTOM:
        m_posFullLiftStage1=Constants.Lift.posLowFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posLowFullLiftStage2;
        m_angleStage1ABS=Constants.Rotate.angleLowConeStage1ABS;
        m_angleStage2ABS=Constants.Rotate.angleLowConeStage2ABS;
      case PLAYERSTATION:
       m_posFullLiftStage1 = Constants.Lift.posPlayerLiftStage1;
       m_posFullLiftStage2 = Constants.Lift.posPlayerLiftStage2;
       m_angleStage1ABS = Constants.Rotate.anglePlayerStage1ABS;
       m_angleStage2ABS = Constants.Rotate.anglePlayerStage2ABS;
      break;
    }
    m_posInitLift=Constants.Lift.posInitLift;
     
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.INITLIFT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //System.out.println("Move Arm Extend " + m_state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("Ending Auto Score");
      m_elevator.stop();
      m_rotate.stopRotate();
      }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean returnValue=false;
    System.out.println(m_state);
    switch(m_state){
      case INITLIFT:
          
          // Encoder is negative as it lifts up
          if(m_elevator.getElevatorPos()>m_posInitLift){
            m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage1Speed);
          }else{
            m_elevator.stop();
            m_state=STATE.EXTENDANDROTATE;
          // m_state=STATE.ABORT;
          }
          returnValue=false;
          break;
      case ROTATE:
         if(m_rotate.getAbsolutePos()<m_angleStage2ABS){
          m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);
         } else{
          System.out.println("Rotate ABS on Stop - " + m_rotate.getAbsolutePos());
          m_rotate.stopRotate();
          m_state=STATE.EXTENDLIFT;
         }
         returnValue=false;
          break;
      case EXTENDLIFT:
       // Encoder is negative as it lifts up
       if(m_elevator.getElevatorPos()>m_posFullLiftStage2){
        m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage2Speed);
      } else{
        if(m_elevator.getElevatorPos()<m_posFullLiftStage2){
          m_elevator.runWithOutLimit(-Constants.Lift.liftAutoExtendStage2Speed);
        }else{
          m_elevator.stop();
          m_state=STATE.END;
        }
      }
     
        returnValue=false;
        break;
      case EXTENDANDROTATE:
       boolean extended=false;
       boolean rotated=false;
       double currentRotateEncoder=m_rotate.getAbsolutePos();
       System.out.println("Current Rotate" + currentRotateEncoder);
      if(currentRotateEncoder>m_angleStage1ABS){
        m_rotate.rotate(Constants.Rotate.rotateAutoOutStage1Speed);
       } else{
          if(currentRotateEncoder>m_angleStage2ABS){
            m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);  
            } else{
              m_rotate.stopRotate();
              rotated=true;
             }
      }
      
       if(m_elevator.getElevatorPos()>m_posFullLiftStage1){
        m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage1Speed);
      } else{
          if(m_elevator.getElevatorPos()>m_posFullLiftStage2){
            m_elevator.runWithOutLimit(Constants.Lift.liftAutoExtendStage2Speed);
          }else{
            if(m_elevator.getElevatorPos()<m_posFullLiftStage2){
              System.out.println("Past Target");
              if(Math.abs(m_elevator.getElevatorPos()-m_posFullLiftStage2) > kTolerance){
                System.out.println("Running Back");
                m_elevator.runWithOutLimit(-Constants.Lift.liftAutoExtendStage2Speed);
              } else{
                  System.out.println("Past Target but within Tolerance" + m_elevator.getElevatorPos());
                  m_elevator.stop();
                  extended=true;
              }
            }else{
              System.out.println("Stopping 1");
              m_elevator.stop();
            }    
          }
        }
      if(rotated && extended){
        System.out.println("Ending Auto Score");
        m_state=STATE.END;
      }
      break;
      case FINALROTATE:
      //kk 3/4 commented out for playoffs
      m_state=STATE.END;
      /*  if(m_targetLevel==Constants.TargetHeight.PLAYERSTATION){
        m_state=STATE.END;
       }else{
        if(m_rotate.getAbsolutePos() > m_angleStage2ABS-0.05){
          m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);
        }else {
          m_rotate.stopRotate();
          m_state=STATE.END;
        }
       }*/
      break;
      case ABORT:
      case END:
      System.out.println("Ending Auto Score");
        m_elevator.stop();
        m_rotate.stopRotate();
        returnValue=true;

    }
    
    return returnValue;
  }
}
