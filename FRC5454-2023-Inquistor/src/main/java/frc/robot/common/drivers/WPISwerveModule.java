package frc.robot.common.drivers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class WPISwerveModule {
    private double m_wheelRadius=2.0;
    private double m_maxAngularSpeed=Math.PI;
    private double m_maxAngularAcceleration=2 * Math.PI;
    private int m_encoderResolution=4096;

    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor; 

    private RelativeEncoder m_driveEncoder;
    private AnalogInput m_turningEncoder;

    private PIDController m_drivePIDController=new PIDController(1,0,0);
    private ProfiledPIDController m_turningPIDController=new ProfiledPIDController(1,0,0,
                                                         new TrapezoidProfile.Constraints(m_maxAngularSpeed,m_maxAngularAcceleration));

    private SimpleMotorFeedforward m_driveFF=new SimpleMotorFeedforward(1,0.1);
    private SimpleMotorFeedforward m_turnFF=new SimpleMotorFeedforward(1,0.1);

    public WPISwerveModule(int driveMotorPort,int turningMotorPort, AnalogInput turningEncoder){
        m_driveMotor=new CANSparkMax(driveMotorPort,MotorType.kBrushless);
        m_turningMotor=new CANSparkMax(turningMotorPort,MotorType.kBrushless);
        m_driveEncoder=m_driveMotor.getEncoder();
        m_turningEncoder=turningEncoder;

        m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(m_driveEncoder.getPosition(),new Rotation2d(readAngle()));
    }

    public void setState(SwerveModuleState desiredState){ 
        SwerveModuleState state=SwerveModuleState.optimize(desiredState,new Rotation2d(readDistance()));

        double driveOutput=m_drivePIDController.calculate(m_driveEncoder.getVelocity(),state.speedMetersPerSecond);
        double driveFF=m_driveFF.calculate(state.speedMetersPerSecond);

        double turnOutput=m_turningPIDController.calculate(readAngle(),state.angle.getRadians());
        double turnFF=m_turnFF.calculate(m_turningPIDController.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput+driveFF);
        m_turningMotor.setVoltage(turnOutput+turnFF);
    }

    protected double readAngle() {
        double angle = (1.0 - m_turningEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    protected double readDistance() {
        double driveRotations = m_driveEncoder.getPosition();
            
        double driveDistance = driveRotations * (1.0 / m_encoderResolution);
        return driveDistance;
    }
}
