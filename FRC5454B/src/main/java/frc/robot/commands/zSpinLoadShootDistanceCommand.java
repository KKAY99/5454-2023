package frc.robot.commands;


import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.classes.Limelight;

import frc.robot.subsystems.*;
/** An example command that uses an example subsystem. */
public class zSpinLoadShootDistanceCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})  
  private final ShooterSubsystemVoltage m_shooter;
  private final ConveyorSubsystem m_conveyor;
  private final FeederSubsystem m_feeder;
  private final Limelight m_limelight;
  private double m_topSpeed;
  private double m_bottomSpeed;
  private double m_minVelocity;
  private boolean m_isFinished=false;
   
  public zSpinLoadShootDistanceCommand(ShooterSubsystemVoltage shooter, ConveyorSubsystem conveyor,FeederSubsystem feeder,Limelight limelight) {
    m_shooter=shooter;
    m_conveyor=conveyor;
    m_feeder=feeder;
    m_limelight=limelight;
    addRequirements(shooter);
    addRequirements(conveyor);
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_shooter.isUptoSpeedbyDistance(m_limelight.getDistance())==false){
        System.out.println("B-" + m_shooter.getBottomMotorVelocity() + " T-" + m_shooter.getTopMotorVelocity());
        m_shooter.shootbyDistance(m_limelight.getDistance()); 
        m_feeder.run(-Constants.FeederSpeed); // Push balls down
        m_conveyor.stop();
    } else {
        System.out.println("Shooting Now");
        m_shooter.shootbyDistance(m_limelight.getDistance()); 
        m_feeder.run(Constants.FeederSpeed);
        m_conveyor.run(Constants.conveyorUpSpeed);

    }
     }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping zAutoLoadShoot");
    //allow shooter to keep running for auto
          m_conveyor.stop();
      m_feeder.stop();
      m_shooter.stopShooting();
    
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

