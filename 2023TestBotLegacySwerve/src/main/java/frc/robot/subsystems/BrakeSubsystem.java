package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.SparkMaxPIDController;

public class BrakeSubsystem extends SubsystemBase{
    private static CANSparkMax m_RotateMotor;
    private SparkMaxPIDController m_pidController;
    private RelativeEncoder m_relativeEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    
    public BrakeSubsystem(){
        m_RotateMotor = new CANSparkMax(50, MotorType.kBrushless);
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);  
        
        m_relativeEncoder = m_RotateMotor.getAlternateEncoder(kAltEncType, kCPR);

        m_pidController = m_RotateMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_relativeEncoder);
        
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    }

    public double GetPos(){
        double pos = m_relativeEncoder.getPosition();

        return pos;    
    }

    public void SetPosAndMove(double targetPos){
        m_pidController.setReference(targetPos, CANSparkMax.ControlType.kPosition);
    }

}
