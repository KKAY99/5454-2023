package frc.robot.commands;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class zAutoTargetToColumnCommand extends CommandBase{
    private Limelight m_limeLight;
    private DrivetrainSubsystem m_drive;
    private int m_pipeline;
    private double m_targetHeight;
    private double m_targetColumnDistance;
    
    public zAutoTargetToColumnCommand(Limelight limelight,DrivetrainSubsystem drive,int gridChoice){
        m_limeLight = limelight;
        m_drive = drive;
        switch(gridChoice)
        {
            case Constants.ChargedUp.GridPosUpperLeft:
            case Constants.ChargedUp.GridPosMiddleLeft:
            case Constants.ChargedUp.GridPosBottomLeft:
            m_targetColumnDistance = Constants.ChargedUp.leftTargetPositionX;
            break;
            case Constants.ChargedUp.GridPosUpperRight:
            case Constants.ChargedUp.GridPosMiddleRight:
            case Constants.ChargedUp.GridPosBottomRight:
            m_targetColumnDistance = Constants.ChargedUp.rightTargetPositionX;
            break;
            default:
                // Assume AprilTag if not a Cone Position
                m_targetColumnDistance = Constants.ChargedUp.middleTargetPositionX;
            

        }
        switch(gridChoice)
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
            default:
                // Assume AprilTag if not a Cone Position
                m_pipeline=Constants.VisionPipelines.AprilTag;
                m_targetHeight=Constants.ChargedUp.targetHeightAprilTag;
            
        }
      
    }

    @Override
    public void execute(){
        m_limeLight.setPipeline(0);
    }

    @Override
    public boolean isFinished(){
        boolean returnValue = false;
        boolean m_forward = false;
        boolean m_back = false;
        boolean m_right = false;
        boolean m_left = false;
        
        if(m_limeLight.getX()>m_targetColumnDistance){
            m_right = true;

        }else{
            if (m_limeLight.getX()<m_targetColumnDistance){
                m_left = true;
            }
        }
            
        if(m_limeLight.getArea()<Constants.ChargedUp.distanceFromTag){
            m_forward = true;
        }else{
            if(m_limeLight.getArea()>Constants.ChargedUp.distanceFromTag){
                m_back = true;
            }
        }
        if(m_limeLight.isTargetAvailible()){
            if(m_forward||m_back||m_right||m_left){
                    if(m_left){
                        if(m_forward){
                            m_drive.movenodistance(45,0,0.1);
    
                        }else if(m_back){
                            m_drive.movenodistance(135,0,0.1);
    
                        }else{
                            m_drive.movenodistance(90,0,0.1);
    
                        }
                    }else{
                        if(m_right){
                            if(m_forward){
                                m_drive.movenodistance(315,0,0.1);
                
                               }else if(m_back){
                                m_drive.movenodistance(235,0,0.1);
                
                               }else{
                                m_drive.movenodistance(270,0,0.1);
                
                               }
        
                        }else{
                            if(m_forward){
                                m_drive.movenodistance(0,0,0.1);
                            }else{
                                m_drive.movenodistance(180,0,0.1);
                            }
                        }
        
                    }
                    returnValue = false;
                }else{
                    System.out.println("Is Not moving");
                    m_drive.stop();
                    returnValue = true; 
                    
                }
        }else{
            m_drive.stop();
            returnValue = true; 
        }
        return returnValue;
    }
}
