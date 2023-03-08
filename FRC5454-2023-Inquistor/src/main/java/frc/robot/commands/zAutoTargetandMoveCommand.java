package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;



public class zAutoTargetandMoveCommand extends CommandBase{
private int m_gridChoice=0;
private Limelight m_limeLight;
private int m_pipeline=0;
private double m_targetHeight;
private DrivetrainSubsystem m_drive;
private final MedianFilter m_filter = new MedianFilter(3);
private PIDController m_pidRight = new PIDController(Constants.PIDSteering.rightKP,PIDSteering.rightKI,PIDSteering.rightKD);
private PIDController m_pidLeft = new PIDController(Constants.PIDSteering.leftKP,PIDSteering.leftKI,PIDSteering.leftKD);
private double kVisionXTolerance=Constants.LimeLightValues.kVisionXTolerance;
private double kGyroTolerance=5;
private double kTimeToMoveForward=1; // how long do we move forward
private double kMoveSpeed=0.1;
private double m_startMoveTime;
private STATE m_state;

private enum STATE{
  MAKESTRAIGHT,MOVEFORWARD,LINEUP,END,ABORT;
}



public zAutoTargetandMoveCommand(Limelight limelight,DrivetrainSubsystem drive,int gridChoice){
        m_gridChoice=gridChoice;
        m_limeLight=limelight;
        m_drive=drive;
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
    }
      m_state = STATE.MOVEFORWARD;
      m_startMoveTime=0;
         //if the command doesnt have the target abort
         if(!m_limeLight.isTargetAvailible()){
          System.out.println("Lost Target - TargetAvailable=" + m_limeLight.isTargetAvailible());
          m_state=STATE.ABORT;
        }
      
    }
  
     @Override
    public void execute() {
     
    }
    
    
  
    @Override
    public void end(boolean interrupted) {

      m_state = STATE.ABORT;
    }
  
    @Override
    public boolean isFinished() {
   
      boolean returnValue = false;
      double measurement;
      double filteredMeasurement; 
      
      m_limeLight.update();
      measurement = m_limeLight.getXRaw();
   
      filteredMeasurement = m_filter.calculate(measurement);
   
      switch(m_state){
        case MAKESTRAIGHT:
          m_state=STATE.MOVEFORWARD;
         //TODO: Validate works before reneabling 
         /*
          double currentAngle=m_drive.getGyroAngle();
          if(currentAngle<kGyroTolerance){
            m_drive.stop();
            m_state=STATE.MOVEFORWARD;
          }else{
            if(currentAngle<0){
                m_drive.spinLeft(0.10);
            }else{
                m_drive.spinRight(0.10);
              }
          }
           */

        

          break;
          case MOVEFORWARD:
             if(m_startMoveTime==0){
                 System.out.println("AutoMove Forward Started");
                 m_startMoveTime=Timer.getFPGATimestamp();
                 m_drive.movenodistance(0,0,kMoveSpeed);               
              } else {
                if(Timer.getFPGATimestamp()>(m_startMoveTime+kTimeToMoveForward)){
                  System.out.println("AutoMove Forward Stopping " + Timer.getFPGATimestamp() + " " + m_startMoveTime);  
                  m_drive.stop();
                    m_state=STATE.LINEUP;
                }
              }
              break;
              case LINEUP:
                if(Math.abs(filteredMeasurement)<kVisionXTolerance){
                  System.out.println("stopping -filterednumber:" + filteredMeasurement + " lastvalue:" + measurement);
                  m_drive.stop();
                  m_state = STATE.END; 
                }else{
                  if(filteredMeasurement>0){
                  double pidOutput = m_pidRight.calculate(filteredMeasurement);
                  pidOutput=Math.min(Math.max(pidOutput,-0.10),0.10);
                  //System.out.println("Aligning - X is " + m_limeLight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
                  System.out.println("move right " + pidOutput);
                
                  m_drive.movenodistance(90,0,pidOutput);
                }else{
                  double pidOutput = m_pidLeft.calculate(filteredMeasurement);
                  pidOutput=Math.min(Math.max(pidOutput,-0.10),0.10);
                
                  //System.out.println("Aligning - X is " + m_limeLight.getXRaw() + " filtered is " + filteredMeasurement + " pidOutput is " + pidOutput);
                  System.out.println("move left " + pidOutput);
              
                  m_drive.movenodistance(270,0,pidOutput);
                }     
          }
        break;
        case ABORT:
          System.out.println("Aborting zAutoTargetMoveCommand");
        case END:
        m_drive.stop();
        returnValue=true;
      }
      return returnValue; 
  }
    
}
