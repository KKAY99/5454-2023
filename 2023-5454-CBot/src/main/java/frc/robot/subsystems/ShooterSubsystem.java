package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonSRX m_leftShootMotor;
    private TalonSRX m_rightShootMotor;
    private TalonSRX m_leftSnowMotor;
    private TalonSRX m_rightSnowMotor;
    private double m_shootMotorSpeed;
    private double m_snowMotorSpeed;
    private double m_maxSnowVoltage;
    private static final double kIntakeStallVoltage=13;
    public ShooterSubsystem(int leftShootMotorPort, int rightShootMotorPort, int leftSnowMotorPort, int rightSnowMotorPort,double snowMotorSpeed){
        m_leftShootMotor = new TalonSRX(leftSnowMotorPort);
        m_rightShootMotor = new TalonSRX(rightSnowMotorPort);
        m_leftSnowMotor =  new TalonSRX(leftSnowMotorPort);
        m_rightSnowMotor = new TalonSRX(rightSnowMotorPort);
        
        m_snowMotorSpeed=snowMotorSpeed;
    }

    public void intakeCube(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,-shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,-shootMotorSpeed);
        m_leftSnowMotor.set(ControlMode.PercentOutput, -m_snowMotorSpeed);
        m_rightSnowMotor.set(ControlMode.PercentOutput,-m_snowMotorSpeed);
        
        double currentVoltage=m_leftSnowMotor.getMotorOutputVoltage();
        if(currentVoltage>m_maxSnowVoltage){
            m_maxSnowVoltage=currentVoltage;
        }
    }

    public void spinUpShooterMotors(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
    }

    public void expellCube(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_leftSnowMotor.set(ControlMode.PercentOutput, m_snowMotorSpeed);
        m_rightSnowMotor.set(ControlMode.PercentOutput, m_snowMotorSpeed);
    }
   
    public double getMaxSnowVoltage(){
        return m_maxSnowVoltage;
    }
    public boolean hasCube(){
        if)m_maxSnowVoltage>kIntakeStallVoltage){
            return true;
        }else {
            return false;
        }
        
    }
    public void stop(){
        m_leftShootMotor.set(ControlMode.PercentOutput,0);
        m_rightShootMotor.set(ControlMode.PercentOutput,0);
        m_leftSnowMotor.set(ControlMode.PercentOutput, 0);
        m_rightSnowMotor.set(ControlMode.PercentOutput, 0);
        m_maxSnowVoltage=0; // reset max voltage
    } 
    
   
    
    
}
