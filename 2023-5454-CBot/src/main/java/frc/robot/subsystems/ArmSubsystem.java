package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax m_leftArmMotor;
    //private CANSparkMax m_rightArmMotor;
    DutyCycleEncoder m_leftAbsoluteEncoder;
    //RelativeEncoder m_rightRelativeEncoder;
    private double m_homePos;
    public double m_rotateSpeed;
    


    public ArmSubsystem(int leftArmMotorport,int encoderPort, double homePos){

    m_leftArmMotor = new CANSparkMax(1 , MotorType.kBrushed);   
    m_leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftArmMotor.setSmartCurrentLimit(30);
    m_leftAbsoluteEncoder = new DutyCycleEncoder(encoderPort);

    // m_rightArmMotor = new CANSparkMax(1 , MotorType.kBrushed);   
    // m_rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // m_rightArmMotor.setSmartCurrentLimit(30);
    //m_rightRelativeEncoder= m_leftArmMotor.getEncoder();

        m_homePos=homePos;


    }

    public double getEncoderPos(){
        return m_leftAbsoluteEncoder.getAbsolutePosition();
    }

    public void rotateArm(double rotateSpeed){
        m_rotateSpeed=rotateSpeed;
        m_leftArmMotor.set(rotateSpeed);
    }

    public void stopRotate(){
        rotateArm(0);
    }

    public void setHomePos(){
        m_homePos=m_leftArmMotor.getEncoder().getPosition();
    }
    
     public boolean goToPos(double pos){
        boolean isAtPos;
        
     if(pos==getEncoderPos()){
        isAtPos = true;
        stopRotate();   
     }else{
        if(pos<getEncoderPos()){
            isAtPos=false;
            rotateArm(0.2);
        }else{
            isAtPos=false;
             rotateArm(-0.2);
        }
     } 
     return isAtPos;
    }
}