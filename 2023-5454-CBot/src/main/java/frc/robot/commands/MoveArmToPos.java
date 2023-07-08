package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPos extends CommandBase  {
    ArmSubsystem m_Subsystem;
    ArmSubsystem m_shootPos1;
    ArmSubsystem m_ShootPos2;
    ArmSubsystem m_intakePos;
    ArmSubsystem m_leftRelativeEncoder;

    public MoveArmToPos(ArmSubsystem subsystem, ArmSubsystem shootPos1, ArmSubsystem shootPos2, ArmSubsystem intakePos){}
}
