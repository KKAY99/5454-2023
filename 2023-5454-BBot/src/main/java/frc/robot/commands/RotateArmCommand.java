package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;

public class RotateArmCommand extends CommandBase{
    
    private RotateArmSubsystem m_rotateSubsystem;
    private double m_speed;

    public RotateArmCommand(RotateArmSubsystem rotateSubsystem,double speed){
        m_rotateSubsystem=rotateSubsystem;

        m_speed=speed;
    }

    @Override
    public void execute(){
        m_rotateSubsystem.runWithLimits(m_speed);
    }

    @Override
    public void end(final boolean interrupted){
        m_rotateSubsystem.stop();
    }
}
