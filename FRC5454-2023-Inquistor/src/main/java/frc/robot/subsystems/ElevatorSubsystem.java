// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  CANSparkMax m_eMotor;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(Integer MotorPort) {
   /*  m_Motor = new CANSparkMax(IntakeMotorPort, MotorType.kBrushed);   
    m_Motor.setOpenLoopRampRate(0.25);
    m_Motor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_IMotor.setSecondaryCurrentLimit(30); //Set as well at 30
  */}
  public void run(double power) {
    //m_Motor.set(power);
    
  }

  public void stope() {
    //m_Motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
