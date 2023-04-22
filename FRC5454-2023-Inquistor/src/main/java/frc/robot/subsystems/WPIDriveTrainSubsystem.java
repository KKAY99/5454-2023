package frc.robot.subsystems;
import frc.robot.common.drivers.WPISwerveModule;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.drivers.NavX;

public class WPIDriveTrainSubsystem extends SubsystemBase{
    private double m_maxSpeed=3;
    private static final double TRACKWIDTH = 25;
    private static final double WHEELBASE = 31;

    /*private AnalogInput m_frontLeftEncoder=new edu.wpi.first.wpilibj.AnalogInput(Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER);
    private AnalogInput m_frontRightEncoder=new edu.wpi.first.wpilibj.AnalogInput(Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER);
    private AnalogInput m_backLeftEncoder=new edu.wpi.first.wpilibj.AnalogInput(Constants.RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER);
    private AnalogInput m_backRightEncoder=new edu.wpi.first.wpilibj.AnalogInput(Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER);
    */
    private NavX m_gyro;

    private SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    /*private WPISwerveModule m_frontLeftModule=new WPISwerveModule(Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
    Constants.RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,m_frontLeftEncoder);
    private WPISwerveModule m_frontRightModule=new WPISwerveModule(Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
    Constants.RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,m_frontRightEncoder);
    private WPISwerveModule m_backLeftModule=new WPISwerveModule(Constants.RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
    Constants.RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,m_backLeftEncoder);
    private WPISwerveModule m_backRightModule=new WPISwerveModule(Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
    Constants.RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,m_backRightEncoder);*/

    public WPIDriveTrainSubsystem(NavX gyro){
        m_gyro=gyro;
    }

    public void drive(double forwardSpeed, double directionalSpeed, double rotation, boolean fieldCentric){
        var swerveModuleStates= m_driveKinematics.toSwerveModuleStates(fieldCentric?ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed,
                                                                    directionalSpeed,rotation,m_gyro.getAngleInRotation2D()):
                                                                    new ChassisSpeeds(forwardSpeed, directionalSpeed, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
        /*m_frontLeftModule.setState(swerveModuleStates[0]);
        m_frontRightModule.setState(swerveModuleStates[1]);
        m_backLeftModule.setState(swerveModuleStates[2]);
        m_backRightModule.setState(swerveModuleStates[3]);*/
    }
}
