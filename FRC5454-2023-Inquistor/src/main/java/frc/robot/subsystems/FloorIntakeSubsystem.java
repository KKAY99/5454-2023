package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class FloorIntakeSubsystem extends SubsystemBase{
    CANSparkMax m_intakeMotor;
    CANSparkMax m_rotateMotor;

    public FloorIntakeSubsystem(){
        m_intakeMotor = new CANSparkMax(42, MotorType.kBrushed);
        m_intakeMotor.setOpenLoopRampRate(0.25);
        m_intakeMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
        m_intakeMotor.setSecondaryCurrentLimit(30); //Set as well at 30
        m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
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
        return 0;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
