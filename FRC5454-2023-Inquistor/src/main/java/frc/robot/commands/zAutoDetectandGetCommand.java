package frc.robot.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

public class zAutoDetectandGetCommand extends CommandBase {
  private Limelight m_limelight;
  private DrivetrainSubsystem m_drive;

      //TODO: Write this actual Command
    public zAutoDetectandGetCommand(Limelight limelight,DrivetrainSubsystem drive, int elementType){
      m_limelight = limelight;
      m_drive = drive;
   
    }

    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
    m_limelight.setPipeline(3);
    if(m_limelight.isTargetAvailible() == true){
      //rotate move
      m_drive.move(90, 90, 0, 1, false);
      
      //runintake

      //move back
      m_drive.move(180, 0, 0, 1, false);

      //rotate back
      m_drive.move(0, 90, 0, 1, false);
    }else{
      m_limelight.setPipeline(4);
      if(m_limelight.isTargetAvailible() ==true){
        //rotate move
        m_drive.move(90, 90, 0, 1, false);
        
        //runintake

        //move back
        m_drive.move(180, 0, 0, 1, false);
 
        //rotate back
        m_drive.move(0, 90, 0, 1, false);
      }else{
        //rotate move
        m_drive.move(270, 270, 0, 1, false);
        
        //runintake

        //move back
        m_drive.move(180, 0, 0, 1, false);

        //rotate back
        m_drive.move(0, 270, 0, 1, false);
      }
    }
   }  
  
    @Override
    public void end(boolean interrupted) {
    
    }
  
    @Override
    public boolean isFinished() {
     return false;
    }
}