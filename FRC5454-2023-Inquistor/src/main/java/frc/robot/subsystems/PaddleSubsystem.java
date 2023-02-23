package frc.robot.subsystems;

 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
public class PaddleSubsystem extends SubsystemBase {
  CANSparkMax m_Motor;
  DigitalInput m_limit;
  double m_homeSpeed;
  /** Creates a new ExampleSubsystem. */
  public PaddleSubsystem(Integer MotorPort, Integer limitswitch,double homeSpeed) {
    m_Motor = new CANSparkMax(MotorPort, MotorType.kBrushless);   
    m_Motor.setOpenLoopRampRate(0.25);
    m_Motor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_Motor.setSecondaryCurrentLimit(30); //Set as well at 30
    //m_limit = new DigitalInput(limitswitch);
    m_homeSpeed=homeSpeed;
  }

    private boolean checkLimit(){
      return m_limit.get();
  } 
  public double getPos (){
    return m_Motor.getEncoder().getPosition();
  }

    public  void homePaddle(){
      while(checkLimit()==false){
        m_Motor.set(m_homeSpeed);
      }
      stop();
      m_Motor.getEncoder().setPosition(0);

      }
  public void run(double power) {
    m_Motor.set(power);
    
  }

  public void stop() {
    m_Motor.set(0);
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
