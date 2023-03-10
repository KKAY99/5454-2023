package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.Constants;
import frc.robot.Constants.PIDSteering;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;

public class zBalanceRobotCommand extends CommandBase {

  private DrivetrainSubsystem m_drive;
  private double currentTime;
  private NavX m_gyro;
  private double zerolevel =-.77;
  private boolean m_hasTipped = false;
  private double m_waitDelay = 2;
  private double m_startWaitTime = 0;
  private boolean m_waitAfterBalanceShift = false;
  private boolean m_hasNotTippedOnce = true;
  private final MedianFilter m_filter = new MedianFilter(3);
  private PIDController m_pidForward = new PIDController(Constants.PIDSteering.forwardKP,PIDSteering.forwardKI,PIDSteering.forwardKD);
  private PIDController m_pidBack = new PIDController(Constants.PIDSteering.backwardKP,PIDSteering.backwardKI,PIDSteering.backwardKD);
  /** Creates a new testCommand. */
  public zBalanceRobotCommand(NavX gyro, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gyro = gyro;
    m_drive = drive;

    addRequirements(drive);
  }

  public void setLevel(double level) {
    zerolevel = level;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    double kLowBalanceAngleThresholdDegrees  = -0.1;
    double kHighBalanceAngleThresholdDegrees  = 0.1;


     double xAxisRate            = 0;
     double yAxisRate            = 0;
     double pitchAngleDegrees    = m_gyro.getAxis(Axis.PITCH);
     double rollAngleDegrees     = m_gyro.getAxis(Axis.ROLL);
     boolean m_balanced=false;

     System.out.println("Checking Balance IsFinished - Pitch " + pitchAngleDegrees + " ROLL " + rollAngleDegrees);
     
     // Control drive system automatically, 
     // driving in reverse direction of pitch/roll angle,
     // with a magnitude based upon the angle
     
         if(m_hasTipped){
           if(m_waitAfterBalanceShift){
             double currentTime = Timer.getFPGATimestamp();
             System.out.println(currentTime);
             if(currentTime>(m_startWaitTime+m_waitDelay)){
                 m_waitAfterBalanceShift=false;
             }
           }else{
             if(rollAngleDegrees<kLowBalanceAngleThresholdDegrees){
               m_drive.movenodistance(180,0,0.07);
               System.out.println("Forward " + "RollAngle" + rollAngleDegrees + "Yaw" + m_gyro.getYaw());

             }else if(rollAngleDegrees>kHighBalanceAngleThresholdDegrees){
               m_drive.movenodistance(0,0,0.07);
               System.out.println("Backward " + "RollAngle" + rollAngleDegrees + "Yaw" + m_gyro.getYaw());
               m_hasTipped = false;

             }else{
               //System.out.println("Balanced");
               m_drive.stop();
               m_balanced=true;
             }
           }

           }else{
           if(rollAngleDegrees<0.21 && m_hasNotTippedOnce){
             m_drive.movenodistance(180,0,0.2);
             System.out.println("Driving Up -  " + "RollAngle" + rollAngleDegrees + "Yaw" + m_gyro.getYaw());
           }else{
             //m_drive.move(0,0,0.55,0.3,true);
             System.out.println("Tipped " + "RollAngle" + rollAngleDegrees + "Yaw" + m_gyro.getYaw());
             m_waitAfterBalanceShift = true;
             m_startWaitTime = currentTime;
             m_hasNotTippedOnce = false;
             m_hasTipped = true; 
           }
         }
     //return autoBalanceYMode == false;
     // System.out.println(autoBalanceXMode);
     // System.out.println("Execute Command - " + zerolevel + " * CurRoll " + m_gyro.getAxis(Axis.ROLL) + " * CurPitch "+ m_gyro.getAxis(Axis.PITCH));
     
     if (m_balanced){
      return true;
     } else {
     return false;
    }
 }
}
