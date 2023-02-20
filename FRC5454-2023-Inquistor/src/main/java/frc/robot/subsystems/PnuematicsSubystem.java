package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PnuematicsSubystem extends SubsystemBase{
  /** Creates a new ExampleSubsystem. */
  private edu.wpi.first.wpilibj.Compressor m_Compressor;
  private double m_pressure; 
  private Solenoid m_solenoidClaw;
 
  public PnuematicsSubystem(int nodeID,PneumaticsModuleType pModule, int clawSolenoid) {
  try{
    m_Compressor = new Compressor(nodeID,pModule);
    m_solenoidClaw =new Solenoid(nodeID,pModule,clawSolenoid);
    m_solenoidClaw.setPulseDuration(2);
    m_pressure=m_Compressor.getPressure();
  } catch (Exception e){
    System.out.println("Pneumatics Failure");
    System.out.println("Exception Message: " + e.getMessage());
    System.out.println("StackTrace:" + e.getStackTrace().toString());
  }
}
  

  public void setClaw(boolean value){
  System.out.println("SetClaw Subsystem  - " + value);
   m_solenoidClaw.set(value);
  
  }
  public boolean getClaw(){
  //return true;
     return m_solenoidClaw.get();
  }
  public double getPressure(){
  
      return m_pressure;
  }
   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   //  m_pressure=m_Compressor.getPressure();
   
  }
 
   @Override
   public void simulationPeriodic() {
     // This method will be called once per scheduler run during simulation
   }
 }    





   