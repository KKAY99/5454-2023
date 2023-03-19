package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class FloorIntakeCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private ElevatorSubsystem m_elevator;
    private double m_intakeSpeed;
    private double m_rotateSpeed;
    private STATE m_state;
    private STATE m_origstate;
    private double m_liftHeight;
    private double kRotateTolerance=0.01;
    private double m_timeOut;
    private double m_startTime=0;

    public enum STATE {
        AUTOINTAKE,AUTOEXHAUST,MOVEANDINTAKE,INTAKE,EXHAUST,MOVEHOME,MOVETRANSFER,ABORT,END
    }
    public FloorIntakeCommand(FloorIntakeSubsystem intake, ElevatorSubsystem elevator, double intakeSpeed,double rotateSpeed,double liftHeight, STATE state, double timeOut){
        m_intake = intake;
        m_intakeSpeed = intakeSpeed;
        m_rotateSpeed=rotateSpeed;
        m_state=state;
        m_origstate=state;
        m_elevator=elevator;
        m_liftHeight=liftHeight;
        m_timeOut=timeOut;
        //Allow both flor commands to use intake since thy are using different motors
        //addRequirements(m_intake);
    }

    @Override
    public void execute(){
    }
    @Override
    public void initialize(){
        m_state=m_origstate;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
    }
    private boolean isClawMoved(){
        System.out.println("Is Claw Moved " + m_elevator.getElevatorPos() + "checking against" +(m_liftHeight-10));
        if(m_elevator.getElevatorPos() < m_liftHeight){
               return true;                
       }else{
           //have to travel past height to allow for drift down that still occurs
           m_elevator.SetPosAndMove(m_liftHeight-15);
           m_intake.stopRotate();
           return false;
       }
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        System.out.println(m_state);
        switch(m_state){
            case AUTOINTAKE:
            if(isClawMoved()==true){
                System.out.println("AI-" + m_intake.getRotatePos() + " ** " + m_rotateSpeed);
              if(m_intake.getRotatePos()<Constants.FloorIntake.rotateHandOff){
                 m_intake.rotate(m_rotateSpeed);
             }else{
                  m_state=STATE.MOVEANDINTAKE;
             }
           }
            break;
            case MOVEANDINTAKE:
            m_intake.runIntake(m_intakeSpeed);
            m_intake.rotate(m_rotateSpeed);

            if(m_intake.getRotatePos()<=Constants.FloorIntake.rotateLowLimit){
                m_intake.stopRotate();
                m_state=STATE.INTAKE;
            }
            break;
            case INTAKE:
            if(m_timeOut>0){
                double currentTime=Timer.getFPGATimestamp();
                if(m_startTime==0){
                    m_startTime=Timer.getFPGATimestamp();
                }
                if(currentTime>m_startTime+m_timeOut){
                    return true;
                }else{
                    m_intake.runIntake(m_intakeSpeed);
                }
            }else{
                m_intake.runIntake(m_intakeSpeed);
            }
            break;
            case AUTOEXHAUST:
            if(isClawMoved()==true){
            if(m_intake.getRotatePos()>=Constants.FloorIntake.rotateOuttakePos){
                m_intake.rotate(-m_rotateSpeed);
               System.out.println("rotate down " + m_rotateSpeed);
            }else{
                if(m_intake.getRotatePos()<=Constants.FloorIntake.rotateOuttakePos){
                    m_intake.rotate(m_rotateSpeed);
                System.out.println("rotate up" + m_rotateSpeed);
                }
            }
            if(Math.abs(m_intake.getRotatePos()-Constants.FloorIntake.rotateOuttakePos)<kRotateTolerance){
                m_intake.stopRotate();
                m_state=STATE.EXHAUST;
            }
            }
            break;
            case EXHAUST:
            m_intake.runIntake(-m_intakeSpeed);
            break;
            case MOVEHOME:
            if(m_intake.getRotatePos()>=Constants.FloorIntake.rotateHighLimit){
                m_intake.rotate(-m_rotateSpeed);
            }else{
                if(m_intake.getRotatePos()<=Constants.FloorIntake.rotateHighLimit){
                    m_intake.rotate(m_rotateSpeed); 
                }
            }
            if(m_intake.getRotatePos()==Constants.FloorIntake.rotateHighLimit){
                m_intake.stopRotate();
                m_state=STATE.END;
            }
            break;
            case MOVETRANSFER:
            break;
            case ABORT:
            case END:
            return true;
        }      
        return false;
    }
}
