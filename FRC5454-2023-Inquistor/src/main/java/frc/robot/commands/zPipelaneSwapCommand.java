package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;



public class zPipelaneSwapCommand extends CommandBase{
private Limelight m_limeLight;
private int m_pipeline=0;
private double m_targetHeight;
private boolean m_waitforpipelinereset=false;
private double m_pipelineResetTime=0;
private final double kPipelineResetDelay=0.6;


public zPipelaneSwapCommand(Limelight limelight,int gridChoice){
        m_limeLight=limelight;
        //GRID Choice are 1 to 9 for the three rows in a grid 
        //GRID 10 (top cone any column) 11 (middle cone any column) 12 (botton cone any column)
        //GRID 13 (top cube  column) 14 (middle cube any column), 15 (bottom cube any column)
        switch (gridChoice)
        {
            //fall thorugh cases for same pipelines and heights
            case Constants.ChargedUp.GridPosUpperLeft:
            case Constants.ChargedUp.GridPosUpperRight:
            case Constants.ChargedUp.GridPosUpperConeAny:
                m_pipeline=Constants.VisionPipelines.TopTape;   
                m_targetHeight=Constants.ChargedUp.targetHeightHighTape;
                break;
            case Constants.ChargedUp.GridPosMiddleLeft:
            case Constants.ChargedUp.GridPosMiddleRight:
            case Constants.ChargedUp.GridPosMiddleConeAny:
            case Constants.ChargedUp.GridPosBottomLeft:
            case Constants.ChargedUp.GridPosBottomRight:
            case Constants.ChargedUp.GridPosBottomConeAny:
                m_pipeline=Constants.VisionPipelines.BottomTape;
                m_targetHeight=Constants.ChargedUp.targetHeighMLowTape;
                break;
            case Constants.ChargedUp.playerStation:
                // will need to be 15 inches to the left of AprilTag
                m_pipeline=Constants.VisionPipelines.PlayerStationTag;
                m_targetHeight=Constants.ChargedUp.targetHeightPlayerStationTag;
                break;
            default:
                // Assume AprilTag if not a Cone Position
                m_pipeline=Constants.VisionPipelines.AprilTag;
                m_targetHeight=Constants.ChargedUp.targetHeightAprilTag;        
        }
    }
    @Override
    public void initialize() {
      if(m_limeLight.getPipeline()!=m_pipeline){
        //System.out.println("Switching Pipeline - " + m_pipeline + " from " + m_limeLight.getPipeline());
          m_limeLight.setPipeline(m_pipeline);
          m_limeLight.setTargetHeight(m_targetHeight);
          m_limeLight.update();
          m_waitforpipelinereset=true;
          m_pipelineResetTime=Timer.getFPGATimestamp();
        
    }       
    }
  
     @Override
    public void execute() {
           
    }
    
  
    @Override
    public void end(boolean interrupted) {

    }
  
    @Override
    public boolean isFinished() {
        if(m_waitforpipelinereset){
            if(Timer.getFPGATimestamp()>m_pipelineResetTime+kPipelineResetDelay){
              return true;  // delay is over and pieline should be available
            }
            else {
              m_limeLight.update();
              return false;
            }
        }else { 
        return true; 
        }
  }
    
}
