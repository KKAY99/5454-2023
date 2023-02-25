package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class zMoveArmExtend extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final ElevatorSubsystem m_elevator;
  private final RotateArmSubsystem m_rotate;
  private double m_posInitLift;
  private double m_posFullLiftStage1;
  private double m_posFullLiftStage2;
  private double m_angleStage1;
  private double m_angleStage2;
  private static enum STATE
  {
                  INITLIFT,ROTATE,EXTENDANDROTATE,EXTENDLIFT,ABORT,END
  }
 private STATE m_state=STATE.INITLIFT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmExtend(ElevatorSubsystem elevator, RotateArmSubsystem rotate, Constants.TargetHeight targetLevel) {
    m_elevator = elevator;
    m_rotate = rotate;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
    switch(targetLevel){
      case TOP:
      m_posFullLiftStage1=Constants.Lift.posHighFullLiftStage1;
      m_posFullLiftStage2=Constants.Lift.posHighFullLiftStage2;
      m_angleStage1=Constants.Rotate.angleHighConeStage1;
      m_angleStage2=Constants.Rotate.angleHighConeStage2;
      break;
      case MIDDLE:
        m_posFullLiftStage1=Constants.Lift.posMiddleFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posMiddleFullLiftStage2;
        m_angleStage1=Constants.Rotate.angleMiddleConeStage1;
        m_angleStage2=Constants.Rotate.angleMiddleConeStage2;
      break;
      case BOTTOM:
        m_posFullLiftStage1=Constants.Lift.posLowFullLiftStage1;
        m_posFullLiftStage2=Constants.Lift.posLowFullLiftStage2;
        m_angleStage1=Constants.Rotate.angleLowConeStage1;
        m_angleStage2=Constants.Rotate.angleLowConeStage2;
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
  System.out.println("Move Arm Extend " + m_state.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
          } else{
            m_elevator.stop();
            m_state=STATE.EXTENDANDROTATE;
          // m_state=STATE.ABORT;
          }
          returnValue=false;
          break;
      case ROTATE:
         if(m_rotate.getRotatePos()<m_angleStage2){
          m_rotate.rotate(Constants.Rotate.rotateAutoOutStage2Speed);
         } else{
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
        m_elevator.stop();
        m_state=STATE.END;
      }
     
        returnValue=false;
        break;
      case EXTENDANDROTATE:
       boolean extended=false;
       boolean rotated=false;
      if(m_rotate.getRotatePos()<m_angleStage1){
        m_rotate.rotate(Constants.Rotate.rotateAutoOutStage1Speed);
       } else{
          if(m_rotate.getRotatePos()<m_angleStage2){
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
            
            m_elevator.stop();
            extended=true;;
          }
        }
      if(rotated && extended){
        m_state=STATE.END;
      }
      break;
      case ABORT:
      case END:
        m_elevator.stop();
        m_rotate.stopRotate();
        returnValue=true;

    }
    
    return returnValue;
  }
}
