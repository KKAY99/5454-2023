package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class OpenTopClawCommand extends CommandBase{

    private PneumaticsSubsystem m_pneumatics;

    public OpenTopClawCommand(PneumaticsSubsystem pneumatics){
        m_pneumatics=pneumatics;
    }

    @Override
    public void execute(){
        if(m_pneumatics.getTopClawSolenoidState()==true){
            m_pneumatics.setTopClawSolenoid(false);
        }else{
            m_pneumatics.setTopClawSolenoid(true);
        }
    }
    
}
