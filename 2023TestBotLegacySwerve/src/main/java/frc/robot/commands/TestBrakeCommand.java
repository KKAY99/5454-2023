package frc.robot.commands;

import frc.robot.subsystems.BrakeSubsystem;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.SparkMaxAlternateEncoder;

public class TestBrakeCommand  extends CommandBase{
    private BrakeSubsystem m_brakeSubsystem;
    private SparkMaxPIDController m_pidController;

    public TestBrakeCommand(BrakeSubsystem brakeSubsystem){
        m_brakeSubsystem = brakeSubsystem;
    }

    @Override
    public void execute(){
        //System.out.println(m_brakeSubsystem.GetPos());
        double targetPos = m_brakeSubsystem.GetPos()-0.05;
        System.out.println(targetPos);
        m_brakeSubsystem.SetPosAndMove(targetPos);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
