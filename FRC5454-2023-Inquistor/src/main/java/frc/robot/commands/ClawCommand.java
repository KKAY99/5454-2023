package frc.robot.commands;
import frc.robot.subsystems.PnuematicsSubystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawCommand extends CommandBase {
    private PnuematicsSubystem m_Pnuematics;
    private boolean m_state;
    public ClawCommand(PnuematicsSubystem pnuematics,boolean state){
        m_Pnuematics=pnuematics;
        m_state=state;
       
    }

    @Override
    public void initialize() {
    }  

    @Override
    public void execute() {
        m_Pnuematics.setClaw(m_state);
    }
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
     return true;
    }

}