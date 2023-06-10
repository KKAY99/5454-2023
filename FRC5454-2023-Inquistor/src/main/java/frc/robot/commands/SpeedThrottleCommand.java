package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WPISwerve;
import frc.robot.common.drivers.WPISwerveModule;

public class SpeedThrottleCommand extends CommandBase{
    private double m_throttle;
    private WPISwerveModule m_WPIModule;

    public SpeedThrottleCommand(WPISwerveModule WPIModule, int throttle){
        m_throttle=throttle;
        m_WPIModule=WPIModule;
    }

    @Override
    public void execute(){
        if(m_WPIModule.speedMultiplier==1.5||m_WPIModule.speedMultiplier==0.5){

        }else{
            if(m_throttle==-1){
                m_WPIModule.speedMultiplier=m_WPIModule.speedMultiplier-0.1;
            }else{
                m_WPIModule.speedMultiplier=m_WPIModule.speedMultiplier+0.1;
            }
        }
    }
}
