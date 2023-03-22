package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimitSwitches;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

public class FloorIntakeSubsystem extends SubsystemBase{
    CANSparkMax m_intakeMotor;
    CANSparkMax m_rotateMotor;
    //DutyCycleEncoder m_rotateEncoder;
    RelativeEncoder m_rotateEncoder;
    double m_highLimit;
    double m_lowLimit;
    DigitalInput m_limitSwitch;
    public FloorIntakeSubsystem(int intakeMotorPort,int rotateMotorPort, int limitSwitchport,double rotateLowLimit,double rotateHighLimit){ 
        m_intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushed);
        m_intakeMotor.setOpenLoopRampRate(0.25);
        m_intakeMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
        m_intakeMotor.setSecondaryCurrentLimit(40); //Set as well at 30
        m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
        m_rotateMotor = new CANSparkMax(rotateMotorPort, MotorType.kBrushless);
        m_rotateMotor.setOpenLoopRampRate(0.25);
        m_rotateMotor.setSmartCurrentLimit(20);  // likely gets ignored due to brushed motor
        m_rotateMotor.setSecondaryCurrentLimit(30); //Set as well at 30
        m_rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_rotateMotor.setInverted(true);
        m_rotateEncoder=m_rotateMotor.getEncoder();
    
        m_limitSwitch=new DigitalInput(limitSwitchport);
        m_highLimit=rotateHighLimit;
        m_lowLimit=rotateLowLimit;
    }

    public void runIntake(double power) {
        System.out.println("intake motor set" + power);
      m_intakeMotor.set(power);
    }

    public void rotate(double power){
        if(checkRotateLimits(power)){
            stopRotate();
        }else{
            m_rotateMotor.set(power);
    
        }
    
    }
    public boolean checkRotateLimits(double power){
        return false;
    }
    public boolean xcheckRotateLimits(double power){
        double rotatePosition= m_rotateEncoder.getPosition();
       boolean returnValue=false;
        if(power>0){
            if(rotatePosition<=m_highLimit){
                returnValue=true; //Hit high limit
            } 
        }else {
        //don't check bottom limit as there is a physical hard stop
        //this will allow the robot to still intake if it doesn't reset right
      //      if(rotatePosition>=m_lowLimit){
      //          returnValue=true;
      //      }
        }
        return returnValue;
         
    }
     public void stopIntake() {
        System.out.println("intake motor stopping");
        m_intakeMotor.stopMotor();
   
  }
  public boolean getLimitSwitch(){
    return m_limitSwitch.get();
  }
  public void resetEncodertoZero(){
    m_rotateEncoder.setPosition(0);
  }
    public void stopRotate() {
        m_rotateMotor.stopMotor();
    }

    

    public double getRotatePos(){
        return m_rotateEncoder.getPosition();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
//     System.out.println(" * Pos  " + getRotatePos() + " " + getLimitSwitch());
    }
}
