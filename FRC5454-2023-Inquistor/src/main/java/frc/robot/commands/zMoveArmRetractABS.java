package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;


/** An example command that uses an example subsystem. */
public class zMoveArmRetractABS extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  
  private final ElevatorSubsystem m_elevator;
  private final RotateArmSubsystem m_rotate;
  private static enum STATE
  {
                  RETRACT,RETRACTANDROTATE,ABORT,END
  }
 private STATE m_state=STATE.RETRACT;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public zMoveArmRetractABS(ElevatorSubsystem elevator, RotateArmSubsystem rotate) {
    m_elevator=elevator;
    m_rotate=rotate;
  // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state=STATE.RETRACT;
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
    boolean retracted=false;
    boolean returnValue=false;
    System.out.println(m_state);
    switch(m_state){
      case RETRACT:
      retracted = false;
      if(m_elevator.getElevatorPos()<Constants.Lift.posInitLift){
        m_elevator.runWithOutLimit(Constants.Lift.liftAutoRetractSpeed);
       }else{     
          retracted = true;
       }

       if(retracted){
        m_state = STATE.RETRACTANDROTATE;
       }
      break;
      case RETRACTANDROTATE:
      retracted=false;
      boolean rotated=false;
     if(m_rotate.getAbsolutePos()<Constants.Rotate.ABSHomePos){
       m_rotate.rotate(Constants.Rotate.rotateAutoInSpeed);
      }else{
             m_rotate.stopRotate();
             rotated=true;
      }
      if(m_elevator.getElevatorPos()<Constants.Lift.posInitLiftRetract){
       m_elevator.runWithOutLimit(Constants.Lift.liftAutoRetractSpeed);
      }else{     
           m_elevator.stop();
           retracted=true;;
      }
     if(rotated && retracted){
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
