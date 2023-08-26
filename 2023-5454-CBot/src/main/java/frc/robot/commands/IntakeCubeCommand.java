package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCubeCommand extends CommandBase{
    ShooterSubsystem m_ShooterSubsystem;
    double m_intakeMotorSpeed;
    
public IntakeCubeCommand(ShooterSubsystem shooterSubsystem,double intakeMotorSpeed){
    m_ShooterSubsystem=shooterSubsystem;
    m_intakeMotorSpeed=intakeMotorSpeed;
    
}
public void initialize(){
   
}

@Override
public boolean isFinished(){
boolean returnValue=false;
//returnValue=m_ShooterSubsystem.hasCube(); 
return returnValue;
}

@Override
public void execute(){
    m_ShooterSubsystem.intakeCube(m_intakeMotorSpeed);
   
}
}