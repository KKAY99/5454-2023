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

public class LiftSubsystem extends SubsystemBase {
    private CANSparkMax m_RotateMotor;
    private CANSparkMax m_ElevatorMotor;
    private DutyCycleEncoder m_RotateEncoder;
    private RelativeEncoder m_ElevatorRelativeEncoder;
    public LiftSubsystem(){
        m_RotateMotor = new CANSparkMax(Constants.RotateMotorPort, MotorType.kBrushless);   
        m_ElevatorMotor = new CANSparkMax(Constants.ElevatorMotorPort, MotorType.kBrushless);      
        m_ElevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_ElevatorRelativeEncoder=m_ElevatorMotor.getEncoder();
        //TODO: Need to reset elevator back to zero and setPos
        m_ElevatorRelativeEncoder.setPosition(0);
        m_RotateEncoder= new DutyCycleEncoder(Constants.Encoders.PivotWheelEncoder);

      }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    } 
    public double getElevatorPos(){
      System.out.println("EP-Rotator -- " + m_RotateEncoder.getAbsolutePosition() + " Elevator -- "+  m_ElevatorRelativeEncoder.getPosition());
      return m_ElevatorRelativeEncoder.getPosition();
    
    }
    public double getRotatePos(){
      System.out.println("RP-Rotator -- " + m_RotateEncoder.getAbsolutePosition() + " Elevator -- "+  m_ElevatorRelativeEncoder.getPosition());
      return m_RotateEncoder.getAbsolutePosition();    
    }
    public void rotate(double power){
      System.out.println("Setting Power on Rotate - " + power);
      m_RotateMotor.set(power);
      System.out.println("RO-Rotate Position - " + m_RotateEncoder.getAbsolutePosition());
      
    }

    public void runElevator(double power){
      System.out.println("Setting Power on Elevator - " + power);
      m_ElevatorMotor.set(power);
      
    }
    public void stopElevator(){
      m_ElevatorMotor.set(0);
    }

    public void stopRotate(){
      System.out.println("Rotate Position - " + m_RotateEncoder.getAbsolutePosition());
    
      m_RotateMotor.set(0);

    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
