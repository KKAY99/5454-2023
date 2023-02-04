package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
 Solenoid m_solenoidClaw;
 public static PneumaticsModuleType pModule = PneumaticsModuleType.CTREPCM;
  
  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem() {
   m_solenoidClaw= new Solenoid(pModule, Constants.Claw.port); 
  
  }
  
  public void OpenClaw(){
    m_solenoidClaw.set(false);
 
  }
  public void CloseClaw(){
    m_solenoidClaw.set(true);
 
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
