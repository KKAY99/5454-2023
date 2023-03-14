package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

public class FloorIntakeSubsystem extends SubsystemBase{
    CANSparkMax m_intakeMotor;
    CANSparkMax m_rotateMotor;
    RelativeEncoder m_intakeEncoder;

    public FloorIntakeSubsystem(){ 
        m_intakeMotor = new CANSparkMax(Constants.FloorIntake.intakeMotorPort, MotorType.kBrushed);
        m_intakeMotor.setOpenLoopRampRate(0.25);
        m_intakeMotor.setSmartCurrentLimit(20);  // likely gets ignored due to brushed motor
        m_intakeMotor.setSecondaryCurrentLimit(30); //Set as well at 30
        m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        //Using Alternate Encoder off of Intake Motor Controller
        m_intakeEncoder = m_intakeMotor.getAlternateEncoder(Type.kQuadrature, 8192);

        m_rotateMotor = new CANSparkMax(Constants.FloorIntake.rotateMotorPort, MotorType.kBrushless);
        m_rotateMotor.setOpenLoopRampRate(0.25);
        m_rotateMotor.setSmartCurrentLimit(20);  // likely gets ignored due to brushed motor
        m_rotateMotor.setSecondaryCurrentLimit(30); //Set as well at 30
        m_rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void runIntake(double power) {
        m_intakeMotor.set(power);
    }

    public void rotate(double power){
        m_rotateMotor.set(power);
    }
    
    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void stopRotate() {
        m_rotateMotor.stopMotor();
    }

    public void setRotatePos(double angle){
    }

    public double getRotatePos(){
        return m_intakeEncoder.getPosition();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
