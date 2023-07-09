package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {
    
    private CANSparkMax m_leftArmMotor;
    private CANSparkMax m_rightArmMotor;
    RelativeEncoder m_leftRelativeEncoder;
    //RelativeEncoder m_rightRelativeEncoder;
    private double m_homePos;
    private double m_shootPos1;
    private double m_shootPos2;
    private double m_intakePos;
    public double m_rotateSpeed;
    


    public ArmSubsystem(int leftArmMotorport,int rightArmMotorport, int leftRelativeEncoderPort,  int rightRelativeEncoderPort, double homePos, double shootPos1, double shootPos2, double intakePos){

    m_leftArmMotor = new CANSparkMax(1 , MotorType.kBrushless);   
    m_leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftArmMotor.setSmartCurrentLimit(30);
    m_leftRelativeEncoder= m_leftArmMotor.getEncoder();

    m_rightArmMotor = new CANSparkMax(1 , MotorType.kBrushless);   
    m_rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightArmMotor.setSmartCurrentLimit(30);
    //m_rightRelativeEncoder= m_leftArmMotor.getEncoder();

    m_homePos=homePos;
    m_shootPos1=shootPos1;
    m_shootPos2=shootPos2;
    m_intakePos=intakePos;


    }
    public void rotateArm(double rotateSpeed){
        m_rotateSpeed=rotateSpeed;
        m_leftArmMotor.set(rotateSpeed);
        m_rightArmMotor.set(-rotateSpeed);
    }

    public void stop(){
        rotateArm(0);
    }

    public void setHomePos(){
        m_homePos=m_leftArmMotor.getEncoder().getPosition();
    }

}