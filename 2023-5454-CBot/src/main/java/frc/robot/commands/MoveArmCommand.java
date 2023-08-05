package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase {
    ArmSubsystem m_ArmSubsystem;
    double m_rotateSpeed;
    double m_minLimit;
    double m_maxLimit;
    double m_armPos;
    
public MoveArmCommand(double rotateSpeed, ArmSubsystem armSubsystem, double minLimit, double maxLimit, double armPos){
    m_ArmSubsystem = armSubsystem;
    m_rotateSpeed = rotateSpeed;
    m_minLimit = minLimit;
    m_maxLimit = maxLimit;
    m_armPos = armPos;
}
public void initialize(){
}
@Override
public void execute(){
    if(m_minLimit<m_ArmSubsystem.getEncoderPos()){
        if(m_ArmSubsystem.getEncoderPos()<m_maxLimit)
            m_ArmSubsystem.rotateArm(m_rotateSpeed);
    }else{
        m_ArmSubsystem.stopRotate();
    }

    }

@Override
public void end(boolean interrupted) {
    m_ArmSubsystem.stopRotate();
}
}