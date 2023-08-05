package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ShootCubeCommand extends CommandBase{
    ShooterSubsystem m_ShooterSubsystem;
    double m_startTime;
    double m_wait;
    double m_shootMotorSpeed;
    double m_snowMotorSpeed;
public ShootCubeCommand(ShooterSubsystem shooterSubsystem, double wait, double startTime, double shootMotorSpeed, double snowMotorSpeed){
    m_ShooterSubsystem=shooterSubsystem;
    m_wait=wait;
    m_startTime=startTime;
    m_shootMotorSpeed=shootMotorSpeed;
    m_snowMotorSpeed=snowMotorSpeed;
}
public void initialize(){
    m_startTime = Timer.getFPGATimestamp();
}
@Override
public void execute(){
    m_ShooterSubsystem.spinUpMotors(m_shootMotorSpeed);
    if(Timer.getFPGATimestamp()-m_startTime  < m_wait){
        m_ShooterSubsystem.expellCube(m_shootMotorSpeed, m_snowMotorSpeed);
    }else{
        
    }
}
}