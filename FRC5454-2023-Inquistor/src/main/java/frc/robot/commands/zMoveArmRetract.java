package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;


/** An example command that uses an example subsystem. */
public class zMoveArmRetract extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final ElevatorSubsystem m_elevator;
  private final RotateArmSubsystem m_rotate;
  private static enum STATE
  {
                  RETRACTANDROTATE,RETURNLIFT,ABORT,END
  }
 private STATE m_state=STATE.RETRACTANDROTATE;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmRetract(ElevatorSubsystem elevator, RotateArmSubsystem rotate) {
    m_elevator=elevator;
    m_rotate=rotate;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.RETRACTANDROTATE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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

      case RETRACTANDROTATE:
      boolean retracted=false;
      boolean rotated=false;
     if(m_rotate.getRotatePos()>Constants.Rotate.angleIntakePos){
       m_rotate.rotate(Constants.Rotate.rotateAutoInSpeed);
      } else{
        
             m_rotate.stopRotate();
             rotated=true;
     }
      if(m_elevator.getElevatorPos()<Constants.Lift.posInitLift){
       m_elevator.run(Constants.Lift.liftAutoRetractSpeed);
     } else{     
           m_elevator.stop();
           retracted=true;;
         }
     if(rotated && retracted){
       m_state=STATE.RETURNLIFT;
     }
     break;
     case RETURNLIFT:
    
          // Encoder is negative as it lifts up
          if(m_elevator.getElevatorPos()<Constants.Lift.posHome){
            m_elevator.run(Constants.Lift.liftAutoRetractHomeSpeed);
          } else{
            m_elevator.stop();
            m_state=STATE.END;
          }
          returnValue=false;
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
