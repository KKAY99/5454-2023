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
   public void run(double power) {
     //m_Motor.set(power);
     
   }
 
   public void stop() {
     //m_Motor.set(0);
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





   