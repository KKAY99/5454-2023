package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FloorIntakeSubsystem extends SubsystemBase{
    CANSparkMax m_intakeMotor;
    CANSparkMax m_rotateMotor;

    public FloorIntakeSubsystem(){

    }

    public void runIntake(double power) {
        m_intakeMotor.set(power);
    }

    public void rotate(double power){
        m_intakeMotor.set(power);
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
