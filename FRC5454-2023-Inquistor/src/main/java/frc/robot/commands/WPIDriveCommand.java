package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import javax.lang.model.util.ElementScanner14;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.WPIDriveTrainSubsystem;
import frc.robot.common.Utilities;
import frc.robot.Constants;
import frc.robot.Constants.WPISwerve;
public class WPIDriveCommand extends CommandBase {
   
  private final WPIDriveTrainSubsystem m_WPIdrive;
  //private final SwerveSubsystem m_drive;
  private final DoubleSupplier m_drive_fwd;
  private final DoubleSupplier m_drive_strafe;
  private final DoubleSupplier m_drive_rcw;
  private final BooleanSupplier m_fieldMode;

  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter turnLimiter;
  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward   The control input for driving left
   * @param rotation  The control input for driving right
   */
  public WPIDriveCommand(WPIDriveTrainSubsystem subsystem, DoubleSupplier drive_rcw, DoubleSupplier drive_fwd,
      DoubleSupplier drive_strafe,BooleanSupplier isFieldMode) {
    m_WPIdrive = subsystem;
    m_drive_fwd = drive_fwd;
    m_drive_strafe = drive_strafe;
    m_drive_rcw = drive_rcw;
    m_fieldMode=isFieldMode;
    
    xLimiter=new SlewRateLimiter(8);
    yLimiter=new SlewRateLimiter(8);
    turnLimiter=new SlewRateLimiter(2);

    addRequirements(m_WPIdrive);
  }

  @Override
  public void initialize(){
  }

  @Override
  public void execute() {
    double fwdSpeed=m_drive_fwd.getAsDouble();
    double strafeSpeed=m_drive_fwd.getAsDouble();
    double rotSpeed=m_drive_rcw.getAsDouble()*8;

    fwdSpeed=Math.abs(m_drive_fwd.getAsDouble())>0.05?m_drive_fwd.getAsDouble():0.0;
    strafeSpeed=Math.abs(m_drive_strafe.getAsDouble())>0.05?m_drive_strafe.getAsDouble():0.0;
    rotSpeed=Math.abs(m_drive_rcw.getAsDouble())>0.05?m_drive_rcw.getAsDouble():0.0;

    fwdSpeed=yLimiter.calculate(fwdSpeed)*Constants.WPISwerve.physicalMaxSpeedMetersPerSecond;
    strafeSpeed=xLimiter.calculate(strafeSpeed)*Constants.WPISwerve.physicalMaxSpeedMetersPerSecond;
    rotSpeed=turnLimiter.calculate(rotSpeed)*Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond;

    if(rotSpeed>Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond){
      rotSpeed=Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond;
    }else if(rotSpeed< -Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond){
      rotSpeed= -Constants.WPISwerve.physicalMaxAngularSpeedRadiansPerSecond;
    }

    ChassisSpeeds chassisSpeeds;

    chassisSpeeds=ChassisSpeeds.fromFieldRelativeSpeeds(strafeSpeed,fwdSpeed,rotSpeed,
                                      Rotation2d.fromDegrees(m_WPIdrive.getRobotDegrees()));
                                 
    SwerveModuleState[] moduleStates=m_WPIdrive.m_driveKinematics.toSwerveModuleStates(chassisSpeeds);

    m_WPIdrive.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted){
    m_WPIdrive.stopAllModules();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}