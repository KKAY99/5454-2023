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
    private final MedianFilter m_filter = new MedianFilter(5);
    private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.rightKI,PIDSteering.rightKD);
    private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.leftKI,PIDSteering.leftKD);
        
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
        m_limeLight.setPipeline(Constants.VisionPipelines.AprilTag);
    }

    @Override
    public boolean isFinished(){
        boolean returnValue = false;
        boolean forward = false;
        boolean back = false;
        boolean right = false;
        boolean left = false;
        int direction=0;
        double speed=0;
        int rotation=0; 
        
        if(m_limeLight.getX()>Constants.LimeLightValues.kVisionXTolerance){
            right = true;

        }else{
            if (m_limeLight.getX()<-(0-Constants.LimeLightValues.kVisionXTolerance)){
                left = true;
            }
        }
            
        if(m_limeLight.getArea()<Constants.ChargedUp.distanceFromTag){
            forward = true;
        }else{
            if(m_limeLight.getArea()>Constants.ChargedUp.distanceFromTag){
                back = true;
            }
        }
     //   double measurement = m_limeLight.getXRaw();
     //   double filteredMeasurement = m_filter.calculate(measurement);
     //   double pidOutput = m_pidRight.calculate(filteredMeasurement);
     //   pidOutput=Math.min(Math.max(pidOutput,-0.10),.10);
          
        //determine direction, rotation and speed to move
        if(m_limeLight.isTargetAvailible()){
            if(forward||back||right||left){
                    if(left){
                        if(forward){
                           direction=45;
                           speed=0.1;
                           
                        }else if(back){
                            direction=135;
                            speed=0.1;
                               
                        }else{
                           direction=90;
                           speed=0.1;
                           
    
                        }
                    }else{
                        if(right){
                            if(forward){
                               direction=315;
                               speed=0.1;                                                
                            }else if(back){
                                direction=235;
                                speed=0.1;                               
                               }else{
                                direction=270;
                                speed=0.1;                                               
                               }
        
                        }else{
                            if(forward){
                               direction=0;
                               speed=0.1;                               
                            }else{
                                direction=180;
                                speed=0.1;
                            }
                        }
        
                    }
                    m_drive.movenodistance(direction, rotation, speed);
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
 