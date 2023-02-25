package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;


public class RotateArmSubsystem extends SubsystemBase {
    private CANSparkMax m_RotateMotor;
    RelativeEncoder m_rotateEncoder;
    private boolean m_homed=false;
    private double m_homeAngle;
    private DutyCycleEncoder m_AbsoluteEncoder;

  
    public RotateArmSubsystem(int motorPort,int absoluteEncoderPort,double homeAngle){
        m_RotateMotor = new CANSparkMax(motorPort , MotorType.kBrushless);   
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rotateEncoder = m_RotateMotor.getEncoder();
        m_AbsoluteEncoder= new DutyCycleEncoder(absoluteEncoderPort);
        m_homeAngle=homeAngle;
      }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    //System.out.println("ABS Encoder" + m_AbsoluteEncoder.get()  + " Rel Encoder " + m_rotateEncoder.getPosition());
    }

    public void rotate(double power){
      System.out.println("Setting Power on Rotate - " + power);
      m_RotateMotor.set(power);
      
    }

    public void stopRotate(){
      m_RotateMotor.set(0);

    }
    public double getAbsolutePos(){
      return m_AbsoluteEncoder.get();
    }

    public double getRotatePos(){
      return m_rotateEncoder.getPosition();
    }
    public void SetZero(){
      m_rotateEncoder.setPosition(0);
    }
    public boolean hitHomeAngle(){
      //TODO: IMPLEMENT
      return (getAbsolutePos()==m_homeAngle);
     
    }

    public void setHomed(boolean value){
      m_homed=value;
    }
  
    public boolean hasHomed(){
      return m_homed;
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
