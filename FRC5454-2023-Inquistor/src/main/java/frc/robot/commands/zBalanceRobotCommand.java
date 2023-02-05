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
  private double zerolevel = -.77;
  private boolean m_hasTipped = false;
  private double m_waitDelay = 5;
  private double m_startWaitTime = 0;
  private boolean m_waitAfterBalanceShift = false;
  private boolean m_drivingForward = true;
  private boolean m_moveBack = false;
  private final MedianFilter m_filter = new MedianFilter(1);
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
    System.out.println(m_filter.calculate(m_gyro.getAxis(Axis.ROLL)));
    double balanceThreshold = 0.17;
    double kLowBalanceAngleThresholdDegrees = -balanceThreshold;
    double kHighBalanceAngleThresholdDegrees = balanceThreshold;

    double xAxisRate = 0;
    double yAxisRate = 0;
    double pitchAngleDegrees = m_gyro.getAxis(Axis.PITCH);
    double rollAngleDegrees = m_gyro.getAxis(Axis.ROLL);
    boolean autoBalanceXMode = true;
    boolean autoBalanceYMode = true;

    if (!autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) >= Math.abs(kHighBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = true;
    } else if (autoBalanceXMode &&
        (Math.abs(pitchAngleDegrees) <= Math.abs(kHighBalanceAngleThresholdDegrees))) {
      autoBalanceXMode = false;
    }
    if (!autoBalanceYMode &&
        (Math.abs(rollAngleDegrees) >= Math.abs(kHighBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = true;
    } else if (autoBalanceYMode &&
        (Math.abs(rollAngleDegrees) <= Math.abs(kHighBalanceAngleThresholdDegrees))) {
      autoBalanceYMode = false;
    }

    // Control drive system automatically,
    // driving in reverse direction of pitch/roll angle,
    // with a magnitude based upon the angle
    double measurement = rollAngleDegrees;
    double filteredMeasurement = m_filter.calculate(measurement);
    if(autoBalanceYMode){
      if(m_drivingForward){
        if(filteredMeasurement > balanceThreshold) {
          System.out.println("Driving Up");
          m_drive.movenodistance(180, 0, 0.15);
        } else {
          System.out.println("Tipped");
          m_drive.stop();
          m_drivingForward = false;
          m_moveBack = true;
        }

      }else if(m_moveBack){
        m_drive.move(0,0,0.27,1.5,true);
        m_waitAfterBalanceShift = true;
        m_startWaitTime = currentTime;
        m_moveBack = false;
        m_hasTipped = true;

      }else if(m_hasTipped){
        if(m_waitAfterBalanceShift) {
          double currentTime = Timer.getFPGATimestamp();
          System.out.println(currentTime);
          if (currentTime > (m_startWaitTime + m_waitDelay)) {
            m_waitAfterBalanceShift = false;
          }
        }else {
          if(filteredMeasurement > kHighBalanceAngleThresholdDegrees) {
            m_drive.movenodistance(180, 0, 0.08);

          }else{
           if(filteredMeasurement < kLowBalanceAngleThresholdDegrees) {
            m_drive.movenodistance(0, 0, 0.08);

           }else{
            m_drive.stop();
            System.out.println("Balanced");
           }
         }
        }
      }
    }else{
      m_drive.stop();
    }
    // return autoBalanceYMode == false;
    // System.out.println(autoBalanceXMode);
    // System.out.println("Execute Command - " + zerolevel + " * CurRoll " +
    // m_gyro.getAxis(Axis.ROLL) + " * CurPitch "+ m_gyro.getAxis(Axis.PITCH));
    return false;
  
 }
}
