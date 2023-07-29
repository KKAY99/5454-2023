package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class zMoveArmToPos extends CommandBase {
    double m_targetPos;
    ArmSubsystem m_ArmSubsystem;

     
    public zMoveArmToPos(ArmSubsystem armSubsystem, double shootPos){
        m_ArmSubsystem=armSubsystem;
        m_targetPos=shootPos;
    }
    @Override
    public void execute() {
        m_ArmSubsystem.goToPos(m_targetPos);
    }
    @Override
    public void end(boolean interrupted) {
      m_ArmSubsystem.stopRotate();
    }
}