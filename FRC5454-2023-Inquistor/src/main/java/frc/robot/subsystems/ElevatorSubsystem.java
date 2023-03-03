// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

 
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorSubsystem extends SubsystemBase {
  CANSparkMax m_Motor;
  RelativeEncoder m_elevatorEncoder;
  DigitalInput m_limit;
  private boolean m_homed = false;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem(Integer MotorPort, Integer limitswitchport) {
    m_Motor = new CANSparkMax(MotorPort, MotorType.kBrushless);   
    
    m_Motor.setOpenLoopRampRate(0.25);
    m_Motor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_Motor.setSecondaryCurrentLimit(30); //Set as well at 30
    m_Motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_elevatorEncoder=m_Motor.getEncoder();
    m_limit = new DigitalInput(limitswitchport);
   
  }

  public void runWithOutLimit(double power){
    moveElevator(power, false);
  }

  public void runWithLimit(double power){
    moveElevator(power, true);
  }

  public void moveElevator(double power, boolean checklimit) {
    double speed = power;
    System.out.println("Power" + power);
    if(checklimit && hasHitPhysicalLimitSwitch() && power > 0){
      speed = 0;
      System.out.println("Lower Limit Switch");
    }

    if(checklimit && hasHitMaxLimit() && power < 0){
      speed = 0;
      System.out.println("Higher Limit Switch");
    }

    m_Motor.set(speed);
  }

  public void setHomed(boolean value){
    m_homed=value;
  }
  public boolean hasHomed(){
    return m_homed;
  }

  public void stop() {
    m_Motor.set(0);
  }

  public boolean hasHitPhysicalLimitSwitch(){
    boolean hasLimitBeenHit = m_limit.get();
    boolean returnValue = false;

    if(hasLimitBeenHit){
      returnValue = false;
    }else{
      returnValue = true;
    }

    return returnValue;
  }

  public boolean hasHitMaxLimit(){
    if(getElevatorPos() <= Constants.Elevator.maxLimit){
      return true;
    }else{
      return false;
    }
  }

  public void setZero(){
    m_elevatorEncoder.setPosition(0);
  }

  public double getElevatorPos(){
    return m_elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("ELS" + m_limit.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
