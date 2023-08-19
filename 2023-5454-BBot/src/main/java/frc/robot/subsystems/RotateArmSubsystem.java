package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import com.revrobotics.SparkMaxRelativeEncoder;

public class RotateArmSubsystem extends SubsystemBase{

    private CANSparkMax m_rotateMotor;
    private DutyCycleEncoder m_absEncoder;

    public RotateArmSubsystem(int rotateMotor,int absEncoderPort){
        m_rotateMotor=new CANSparkMax(rotateMotor, MotorType.kBrushed);
        m_absEncoder=new DutyCycleEncoder(absEncoderPort);
    }

    public void runWithLimits(double speed){
        if(hasHitBWDSoftLimit()==false&&hasHitFWDSoftLimit()==false){
            m_rotateMotor.set(speed);
        }else{
            m_rotateMotor.set(0);
        }
    }

    public void runWithOutLimits(double speed){
        m_rotateMotor.set(speed);
    }

    public void stop(){
        m_rotateMotor.stopMotor();
    }

    public double getRotatePos(){
        return m_absEncoder.getDistance();
    }

    public boolean hasHitFWDSoftLimit(){
        boolean hasHitLimit=false;

        return hasHitLimit;
    }

    public boolean hasHitBWDSoftLimit(){
        boolean hasHitLimit=false;

        return hasHitLimit;
    }
}
