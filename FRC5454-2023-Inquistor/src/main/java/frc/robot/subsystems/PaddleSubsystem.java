package frc.robot.subsystems;

 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

public class PaddleSubsystem extends SubsystemBase {
  private CANSparkMax m_Motor;
  private RelativeEncoder m_Encoder;
  private DigitalInput m_limit;
  private double m_homeSpeed;
  private SparkMaxPIDController m_pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private boolean m_homed=false;
  private double m_encoderMoveOutPosStart; 
  private double m_encoderMoveOutPosEnd;  
  /** Creates a new ExampleSubsystem. */
  public PaddleSubsystem(Integer MotorPort, Integer limitswitch,double homeSpeed,double encoderMoveOutPosStart,double encoderMoveOutPosEnd) {
    m_Motor = new CANSparkMax(MotorPort, MotorType.kBrushless);  
    m_Encoder=m_Motor.getEncoder(); 
    m_Motor.setOpenLoopRampRate(0.25);
    m_Motor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_Motor.setSecondaryCurrentLimit(30); //Set as well at 30
    m_limit = new DigitalInput(limitswitch);
    m_encoderMoveOutPosStart=encoderMoveOutPosStart;
    m_encoderMoveOutPosEnd=encoderMoveOutPosEnd;
    /*PADDLE PID
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    m_pidController = m_Motor.getPIDController();
    m_pidController.setFeedbackDevice(m_Encoder);
     
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
   */



    m_homeSpeed=homeSpeed;
  }
 
  public double getPos (){
    return m_Encoder.getPosition();
  }

   
  public void run(double power) {
    m_Motor.set(power);
    
  }
  public void stop() {
    m_Motor.set(0);
  }

  public void moveToPosition(double targetPos){
        double currentPos = getPos();
        
        //m_pidController.setReference(targetPos, CANSparkMax.ControlType.kPosition);
  }
  
  private boolean checkLimit(){
    return m_limit.get();
  }
  public boolean hitPhysicalLimitSwitch(){
    return m_limit.get();
  }
 
  public void SetZero(){
    System.out.println("Set Zero on Paddle");
    m_Encoder.setPosition(0);
  }
  
  public void setHomed(boolean value){
    m_homed=value;
  }
  public boolean checkandMoveTowardsPosition(double targetPos, double speed, double tolerance ){
    boolean returnValue = false;
    double currentPos=getPos();
    double distancefromTarget=targetPos-currentPos;
    double moveSpeed=0;
    //check if position is within tolerance and then stop command
    if(Math.abs(distancefromTarget)<=tolerance){
      stop();
      returnValue=true;
    } else{
        run(moveSpeed);        
      }  
    return returnValue;
  }

  public boolean hasHomed(){
    return m_homed;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // System.out.println("Paddle limit " + m_limit.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
