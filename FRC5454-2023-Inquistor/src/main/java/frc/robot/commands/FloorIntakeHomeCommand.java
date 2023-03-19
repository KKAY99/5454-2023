package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class FloorIntakeHomeCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private double m_timeOut;
    private double m_speed;
    private double m_startTime;
    public FloorIntakeHomeCommand(FloorIntakeSubsystem intake,double speed,double timeOut){
        m_intake = intake;
        m_speed = speed;
        m_timeOut = timeOut;
    }

    @Override
    public void initialize(){
        m_startTime=Timer.getFPGATimestamp();
    }
    
    private void homeFloorintake(){
        m_intake.resetEncodertoZero();
        m_intake.stopRotate();
    }
    
    @Override 
    public boolean isFinished(){
        double currentTime=Timer.getFPGATimestamp();
        if(currentTime>m_startTime+m_timeOut){
            homeFloorintake();
            return true;
        }else{
            if(m_intake.getLimitSwitch()){
               homeFloorintake();
                return true;
            }else{
                m_intake.rotate(-m_speed);
            }

        }
        return false;
    }
}
