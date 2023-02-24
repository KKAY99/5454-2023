// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

public class IntakeArmsSubsystem extends SubsystemBase {
  CANSparkMax m_armMotor;
  CANSparkMax m_armMotor2;

  DigitalInput m_LimitSwitch;
  DigitalInput m_LimitSwitch2;
  double m_ExtendLimit;
  boolean m_homed=false;
  
  private static double kMotorOffsetToAlign=0.15;

  /** Creates a new ExampleSubsystem. */
  public IntakeArmsSubsystem (Integer masterMotorPort,Integer slaveMotorPort, Integer limitswitchPort,double extendLimit) {
    m_armMotor = new CANSparkMax(masterMotorPort, MotorType.kBrushless);   
    m_armMotor.setOpenLoopRampRate(0.25);
    m_armMotor.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_armMotor.setSecondaryCurrentLimit(30); //Set as well at 30
    m_armMotor2 = new CANSparkMax(slaveMotorPort, MotorType.kBrushless);   
    m_armMotor2.setOpenLoopRampRate(0.25);
    m_armMotor2.setSmartCurrentLimit(30);  // likely gets ignored due to brushed motor
    m_armMotor2.setSecondaryCurrentLimit(30); //Set as well at 30
   // m_armMotor2.follow(m_armMotor,true);
    m_LimitSwitch= new DigitalInput(limitswitchPort);
    m_ExtendLimit=extendLimit;
  }

  public void runNOEncoders (double power){
    double arm1Power=power;
    double arm2Power=-power; 
    m_armMotor.set(arm1Power);
    m_armMotor2.set(arm2Power);
     
  }
  public void runNoLimits(double power){
    movearms(power,false);
  
  }
  public void runwithLimits(double power) {
   movearms(power,true);
  }
  private void movearms(double power,boolean checkLimits){
   RelativeEncoder arm1Enocder= m_armMotor.getEncoder();
   RelativeEncoder arm2Encoder=m_armMotor2.getEncoder();
   double arm1Pos=Math.abs(arm1Enocder.getPosition());
   double arm2Pos=Math.abs(arm2Encoder.getPosition());
   double arm1Power=power;
   double arm2Power=-power;
   if(m_homed){
      //negative  is intake out
      if(power<0){
        if(arm1Pos<arm2Pos){
          arm2Power=arm2Power+kMotorOffsetToAlign;  // speed is negative so add to slow it down
          } else{ 
          if(arm2Pos<arm1Pos){
            arm1Power=arm1Power+kMotorOffsetToAlign;  // speed is negative so add to slow it down
          }
        }
      } else {
          if(power>0){
            if(arm1Pos<arm2Pos){
              arm2Power=arm2Power-kMotorOffsetToAlign;  // speed is positive so subtract to slow it down
          } else{ 
              if(arm2Pos<arm1Pos){
                arm1Power=arm1Power-kMotorOffsetToAlign;  // speed is negative so subtract to slow it down
              }
          }
        }
        }
      //check soft limit on if homed only if arm is extending which menas power is greater than zero
      //this lets it ignore extend limit if we are retracting      
      if(checkLimits && (arm1Pos>=m_ExtendLimit)  && (arm1Power>0)){
        arm1Power=0;
        arm2Power=0;
      }
  }
  //intake limit of limit switch
  if(checkLimits && checkLimit1()){
    arm1Power=0;
    arm2Power=0;
  }

   m_armMotor.set(arm1Power);
   m_armMotor2.set(arm2Power);
  }

  public void stop() {
    m_armMotor.set(0);
    m_armMotor2.set(0);
  }
  private boolean checkLimit1(){
    return m_LimitSwitch.get();
  }
  public boolean hitPhysicalLimitSwitch(){
    return checkLimit1();
  }
  public void moveToPosition(double targetPos){
    double currentPos = getPos();
    
    //m_pidController.setReference(targetPos, CANSparkMax.ControlType.kPosition);
}

  public boolean atLimit(double power){
    boolean returnValue=false;
    if(checkLimit1()) {
       returnValue=true;
    } else { // limit switch no hit
      if(m_homed){
        //extract limit only applies if extending which means power>0 
        if((getPos()>m_ExtendLimit || getPos()<=0) && (power>0)){
           returnValue=true;
        }
      }
    }
    return returnValue;
    }
  

    public void SetZero(){
      m_armMotor.getEncoder().setPosition(0);
    }
    
  public double getPos(){
    //USE arm 1 encoder for position of arm
    return m_armMotor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method; will be called once per scheduler run
   // System.out.println("Arm Limit Switch " + checkLimit1() + 
   //                     "Enc 1 "+ m_armMotor.getEncoder().getPosition() +
   //                     " Enc 2 "+ m_armMotor2.getEncoder().getPosition()); 
                                          
  }

  public void setHomed(boolean value){
    m_homed=value;
  }
  public boolean hasHomed(){
    return m_homed;
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
