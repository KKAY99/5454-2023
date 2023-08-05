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

    public ShooterSubsystem(double shootMotorSpeed, double snowMotorSpeed, int leftShootMotorPort, int rightShootMotorPort, int leftSnowMotorPort, int rightSnowMotorPort){
        m_leftShootMotor = new TalonSRX(leftSnowMotorPort);
        m_rightShootMotor = new TalonSRX(rightSnowMotorPort);
        m_leftSnowMotor =  new TalonSRX(leftSnowMotorPort);
        m_rightSnowMotor = new TalonSRX(rightSnowMotorPort);
        
        m_shootMotorSpeed=shootMotorSpeed;
        m_snowMotorSpeed=snowMotorSpeed;
    }

    public void intakeCube(double shootMotorSpeed, double snowMotorSpeed/*, int m_leftShootMotor, int m_rightShootMotor, int m_leftSnowMotor, int m_rightSnowMotor*/){
        m_leftShootMotor.set(ControlMode.PercentOutput,-shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,-shootMotorSpeed);
        m_leftSnowMotor.set(ControlMode.PercentOutput, -snowMotorSpeed);
        m_rightSnowMotor.set(ControlMode.PercentOutput,-snowMotorSpeed);
    }

    public void spinUpMotors(double shootMotorSpeed){
        m_leftShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
    }

    public void expellCube(double shootMotorSpeed, double snowMotorSpeed/*, int m_leftShootMotor, int m_rightShootMotor, int m_leftSnowMotor, int m_rightSnowMotor*/){
        m_leftShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_rightShootMotor.set(ControlMode.PercentOutput,shootMotorSpeed);
        m_leftSnowMotor.set(ControlMode.PercentOutput, snowMotorSpeed);
        m_rightSnowMotor.set(ControlMode.PercentOutput, snowMotorSpeed);
    }

    
    
    
}
