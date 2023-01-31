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


public class RotateArmSubsystem extends SubsystemBase {
    private CANSparkMax m_RotateMotor;
  
    public RotateArmSubsystem(){
        m_RotateMotor = new CANSparkMax(Constants.RotateArm.rotateArmPort , MotorType.kBrushless);   
        m_RotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

      }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void rotate(double power){
      System.out.println("Setting Power on Rotate - " + power);
      m_RotateMotor.set(power);
      
    }

    public void stopRotate(){
      m_RotateMotor.set(0);

    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
