package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

public class FloorIntakeSubsystem extends SubsystemBase{
    CANSparkMax m_intakeMotor;
    CANSparkMax m_rotateMotor;
    DutyCycleEncoder m_rotateEncoder;
    double m_highLimit;
    double m_lowLimit;

    public FloorIntakeSubsystem(int intakeMotorPort,int rotateMotorPort, int rotateEncoder,double rotateLowLimit,double rotateHighLimit){ 
        m_intakeMotor = new CANSparkMax(intakeMotorPort, MotorType.kBrushed);
        m_intakeMotor.setOpenLoopRampRate(0.25);
        m_intakeMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
        m_intakeMotor.setSecondaryCurrentLimit(40); //Set as well at 30
        m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_rotateEncoder=new DutyCycleEncoder(rotateEncoder);
        m_rotateMotor = new CANSparkMax(rotateMotorPort, MotorType.kBrushless);
        m_rotateMotor.setOpenLoopRampRate(0.25);
        m_rotateMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
        m_rotateMotor.setSecondaryCurrentLimit(30); //Set as well at 30
        m_rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_highLimit=rotateHighLimit;
        m_lowLimit=rotateLowLimit;
    }

    public void runIntake(double power) {
        System.out.println("intake motor set" + power);
      m_intakeMotor.set(power);
    }

    public void rotate(double power){
        m_rotateMotor.set(power);
    }
    public boolean checkRotateLimits(double power){
       double rotatePosition= m_rotateEncoder.get();
       boolean returnValue=false;
        if(power>0){
            if(rotatePosition>=m_highLimit){
                returnValue=true; //Hit high limit
            } 
        }else {
            if(rotatePosition<=m_lowLimit){
                returnValue=true;
            }
        }
        return returnValue;
    }
    public void stopIntake() {
        System.out.println("intake motor stopping");
        m_intakeMotor.stopMotor();
    }

    public void stopRotate() {
        m_rotateMotor.stopMotor();
    }

    public void setRotatePos(double angle){
    }

    public double getRotatePos(){
        return m_rotateEncoder.get();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
     System.out.println(" * Pos * " +  m_rotateEncoder.get());
    }
}
