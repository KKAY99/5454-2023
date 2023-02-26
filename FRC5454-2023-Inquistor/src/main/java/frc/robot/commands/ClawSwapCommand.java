package frc.robot.commands;
import frc.robot.subsystems.PnuematicsSubystem;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClawSwapCommand extends CommandBase {
    private PnuematicsSubystem m_Pnuematics;
    public ClawSwapCommand(PnuematicsSubystem pnuematics){
        m_Pnuematics=pnuematics;
    
    
    }

    @Override
    public void initialize() {
       
    }  

    @Override
    public void execute() {
    System.out.println("Swapping Claw");
    if(m_Pnuematics.getClaw()){
        m_Pnuematics.setClaw(false);
    } else{
        m_Pnuematics.setClaw(true);
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