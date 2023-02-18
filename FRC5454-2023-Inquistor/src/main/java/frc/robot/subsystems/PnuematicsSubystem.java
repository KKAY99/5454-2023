package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PnuematicsSubystem extends SubsystemBase{
  /** Creates a new ExampleSubsystem. */
  private edu.wpi.first.wpilibj.Compressor m_Compressor;
  public PnuematicsSubystem(int nodeID) {
    
    //    m_Compressor = new Compressor(nodeID,Constants.Pneumatics.moduleType)  ; 
}

  public void setClaw(boolean value){
    //TODO
  }
  public boolean getClaw(){
      //TODO
      return false;
  }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }
 }    





   