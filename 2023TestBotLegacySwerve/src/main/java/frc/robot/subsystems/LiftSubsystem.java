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
import frc.robot.Constants.Lift;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAlternateEncoder;

public class LiftSubsystem extends SubsystemBase {
    private CANSparkMax m_RotateMotor;
    private CANSparkMax m_ElevatorMotor;
    private DutyCycleEncoder m_RotateEncoder;
    private RelativeEncoder m_ElevatorRelativeEncoder;
    private RelativeEncoder m_RotateAlternateEncoder;
    private RelativeEncoder m_RotateRelativeEncoder;
    private SparkMaxPIDController m_pidController;
    private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
    private static final int kCPR = 8192;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double m_targetPos=0;
    public LiftSubsystem(){
        m_RotateMotor = new CANSparkMax(Constants.RotateMotorPort, MotorType.kBrushless);   
        m_ElevatorMotor = new CANSparkMax(Constants.ElevatorMotorPort, MotorType.kBrushless);      
        m_ElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_ElevatorRelativeEncoder=m_ElevatorMotor.getEncoder();
        //TODO: Need to reset elevator back to zero and setPos
        m_ElevatorRelativeEncoder.setPosition(0);
        m_RotateEncoder= new DutyCycleEncoder(Constants.Encoders.PivotWheelEncoder);

        //m_RotateAlternateEncoder = m_RotateMotor.getAlternateEncoder(kAltEncType, kCPR);
        m_RotateRelativeEncoder=m_RotateMotor.getEncoder(); 
        m_pidController = m_RotateMotor.getPIDController();
       // m_pidController.setFeedbackDevice(m_RotateAlternateEncoder);
        m_pidController.setFeedbackDevice(m_RotateRelativeEncoder);
        
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
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      System.out.println("ABS Pos " + m_RotateEncoder.getAbsolutePosition() + " Current Pos "+ GetPos() + " Target Pos "+ m_targetPos);
    } 
    public double getElevatorPos(){
      //System.out.println("EP-Rotator -- " + m_RotateEncoder.getAbsolutePosition() + " Elevator -- "+  m_ElevatorRelativeEncoder.getPosition());
      return m_ElevatorRelativeEncoder.getPosition();
    
    }
    public double getRotatePos(){
      //System.out.println("RP-Rotator -- " + m_RotateEncoder.getAbsolutePosition() + " Elevator -- "+  m_ElevatorRelativeEncoder.getPosition());
      return m_RotateEncoder.getAbsolutePosition();    
    }
    public void rotate(double power){
      //System.out.println("Setting Power on Rotate - " + power);
      m_RotateMotor.set(power);
      //System.out.println("RO-Rotate Position - " + m_RotateEncoder.getAbsolutePosition());
      System.out.println(m_ElevatorRelativeEncoder.getPosition());
    }

    public void runElevator(double power){
      //System.out.println("Setting Power on Elevator - " + power);
      m_ElevatorMotor.set(power);
      
    }
    public void stopElevator(){
      m_ElevatorMotor.set(0);
    }

    public void stopRotate(){
      //System.out.println("Rotate Position - " + m_RotateEncoder.getAbsolutePosition());
    
      m_RotateMotor.set(0);

    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public double GetPos(){
      double pos = m_RotateRelativeEncoder.getPosition();

      return pos;    
   }

    public void SetPosAndMove(double targetPos){
      m_pidController.setReference(targetPos, CANSparkMax.ControlType.kPosition);
      m_targetPos = targetPos;
    }

    public void InitHomeElevator(){
      double timer = Timer.getFPGATimestamp();
      while(timer != 5){
        while(m_RotateEncoder.getAbsolutePosition() != Lift.absHome){
          if(m_RotateEncoder.getAbsolutePosition() > Lift.absHome){
            runElevator(-0.01);
          }else{
            runElevator(0.01);
          }
        }
        if(m_RotateEncoder.getAbsolutePosition() == Lift.absHome){
          stopElevator();
          m_RotateEncoder.reset();
        }
      }

      if(timer >= 5.0){
        stopElevator();
        m_RotateEncoder.reset();
      }
    }
}
