package frc.robot.common.drivers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;

public class WPISwerveModule {
    private double m_wheelRadius=2.0;
    private double m_maxAngularSpeed=0;
    private double m_maxAngularAcceleration=2 * Math.PI;
    private int m_encoderResolution=0;

    private MotorController m_driveMotor;
    private MotorController m_turningMotor; 

    private Encoder m_driveEncoder;
    private Encoder m_turningEncoder;

    private PIDController m_drivePIDController=new PIDController(1,0,0);
    private ProfiledPIDController m_turningPIDController=new ProfiledPIDController(1,0,0,
                                                         new TrapezoidProfile.Constraints(m_maxAngularSpeed,m_maxAngularAcceleration));

    private SimpleMotorFeedforward m_driveFF=new SimpleMotorFeedforward(1,0);
    private SimpleMotorFeedforward m_turnFF=new SimpleMotorFeedforward(1,0);

    public WPISwerveModule(int driveMotorPort,int turningMotorPort,
                           int driveEncoderA,int driveEncoderB,int turningEncoderA,int turningEncoderB){
        m_driveMotor=new CANSparkMax(driveMotorPort, null);
        m_turningMotor=new CANSparkMax(turningMotorPort, null);
        m_driveEncoder=new Encoder(driveEncoderA,driveEncoderB);
        m_turningEncoder=new Encoder(turningEncoderA,turningEncoderB);

        m_driveEncoder.setDistancePerPulse(2*Math.PI*m_wheelRadius/m_encoderResolution);
        m_turningEncoder.setDistancePerPulse(2*Math.PI*m_wheelRadius/m_encoderResolution);

        m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(m_driveEncoder.getDistance(),new Rotation2d(m_turningEncoder.getDistance()));
    }

    public void setState(SwerveModuleState desiredState){
        SwerveModuleState state=SwerveModuleState.optimize(desiredState,new Rotation2d(m_turningEncoder.getDistance()));

        double driveOutput=m_drivePIDController.calculate(m_driveEncoder.getRate(),state.speedMetersPerSecond);
        double driveFF=m_driveFF.calculate(state.speedMetersPerSecond);

        double turnOutput=m_turningPIDController.calculate(m_turningEncoder.getDistance(),state.angle.getRadians());
        double turnFF=m_turnFF.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput+driveFF);
        m_turningMotor.setVoltage(turnOutput+turnFF);
    }
}
