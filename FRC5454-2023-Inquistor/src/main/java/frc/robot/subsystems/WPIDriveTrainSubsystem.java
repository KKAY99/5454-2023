package frc.robot.subsystems;
import frc.robot.common.drivers.WPISwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.common.drivers.NavX;

public class WPIDriveTrainSubsystem {
    private double m_maxSpeed=0;
    private double m_maxAngularSpeed = Math.PI;
    private static final double TRACKWIDTH = 25;
    private static final double WHEELBASE = 31;

    private NavX m_gyro;

    private SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private WPISwerveModule m_frontLeftModule=new WPISwerveModule(0, 0, 0, 
                                                0, 0, 0);
    private WPISwerveModule m_frontRightModule=new WPISwerveModule(0, 0, 0, 
                                                0, 0, 0);
    private WPISwerveModule m_backLeftModule=new WPISwerveModule(0, 0, 0, 
                                                0, 0, 0);
    private WPISwerveModule m_backRightModule=new WPISwerveModule(0, 0, 0, 
                                                0, 0, 0);

    public WPIDriveTrainSubsystem(NavX gyro){
        m_gyro=gyro;
    }

    public void drive(double forwardSpeed, double directionalSpeed, double rotation, boolean fieldCentric){
        var swerveModuleStates= m_driveKinematics.toSwerveModuleStates(fieldCentric?ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed,
                                                                    directionalSpeed,rotation,m_gyro.getAngleInRotation2D()):
                                                                    new ChassisSpeeds(forwardSpeed, directionalSpeed, rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
        m_frontLeftModule.setState(swerveModuleStates[0]);
        m_frontRightModule.setState(swerveModuleStates[1]);
        m_backLeftModule.setState(swerveModuleStates[2]);
        m_backRightModule.setState(swerveModuleStates[3]);
    }
}
