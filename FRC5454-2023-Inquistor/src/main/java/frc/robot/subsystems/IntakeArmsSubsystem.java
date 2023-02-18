// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmsSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor;
  CANSparkMax m_armMotor2;
  DigitalInput m_LimitSwitch;
  DigitalInput m_LimitSwitch2;

  /** Creates a new ExampleSubsystem. */
  public IntakeArmsSubsystem (Integer MotorPort,Integer MotorPort2, Integer LimitSwitch, Integer LimitSwitch2) {
    m_armMotor = new CANSparkMax(MotorPort, MotorType.kBrushless);   
    m_armMotor.setOpenLoopRampRate(0.25);
    m_armMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_armMotor.setSecondaryCurrentLimit(30); //Set as well at 30
    m_armMotor2 = new CANSparkMax(MotorPort2, MotorType.kBrushless);   
    m_armMotor2.setOpenLoopRampRate(0.25);
    m_armMotor2.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_armMotor2.setSecondaryCurrentLimit(30); //Set as well at 30
    
  }
  public void run(double power) {
    m_armMotor.set(power);
    m_armMotor2.set(power);
  }

  public void stop() {
    m_armMotor.set(0);
    m_armMotor2.set(0);
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
