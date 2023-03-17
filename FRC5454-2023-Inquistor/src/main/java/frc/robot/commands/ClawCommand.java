package frc.robot.commands;
import frc.robot.subsystems.PnuematicsSubystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;

public class ClawCommand extends CommandBase {
    private PnuematicsSubystem m_Pnuematics;
    private Limelight m_limelight;
    private boolean checkForTargets = false;
    private boolean m_state;
    private String m_caller;
    public ClawCommand(PnuematicsSubystem pnuematics,boolean state,String caller){
        m_Pnuematics=pnuematics;
        m_state=state;
        m_caller=caller;
    }

    public ClawCommand(PnuematicsSubystem pnuematics, Limelight limelight,boolean state){
        m_Pnuematics=pnuematics;
        m_limelight = limelight;
        m_caller = "";
        m_state=state;
        checkForTargets = true;
    }

    @Override
    public void initialize() {
     
    }  

    @Override
    public void execute() {
        //System.out.println("Setting Claw to state - " + m_state + " from caller " + m_caller);

        if(checkForTargets){
            if(m_limelight.isTargetAvailible()){
                m_Pnuematics.setClaw(m_state);
            }else{

            }
        }else{
            m_Pnuematics.setClaw(m_state);
        }
    }
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
     return true;
    }

}