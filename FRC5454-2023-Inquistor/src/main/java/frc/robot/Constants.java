 
package frc.robot;


import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;

public final class Constants {
    public static final class ChargedUp {
        public static final double targetHeightAprilTag=18;     // middle of April Tag in Distance
        public static final double targetHeighMLowTape=24.125;  // middle of low tape in inches
        public static final double targetHeightHighTape=43.875; // middle of high tape in inches 
        public static final int GridPosUpperLeft=1;
        public static final int GridPosMiddleLeft=2;
        public static final int GridPosBottomLeft=3;
        public static final int GridPosUpperCenter=4;
        public static final int GridPosMiddleCenter=5;
        public static final int GridPosBottomCenter=6;
        public static final int GridPosUpperRight=7;
        public static final int GridPosMiddleRight=8;
        public static final int GridPosBottomRight=9;
        public static final int GridPosUpperConeAny=10;
        public static final int GridPosMiddleConeAny=11;
        public static final int GridPosBottomConeAny=12;
        public static final int GridPosUpperCubeAny=13;
        public static final int GridPosMiddleCubeAny=14;
        public static final int GridPosBottomCubeAny=15;

        public static final int Cone=0;
        public static final int Cube=1;

        public static final double leftTargetPositionX = 24;
        public static final double middleTargetPositionX = 0;
        public static final double rightTargetPositionX = -15;
        public static final double distanceFromTag = 2.8;
        public static final double AprilTagAlignmentToleranceX=1.5;
        public static final double AprilTagAlignmentToleranceArea=.25;
    }   

    public static final class VisionPipelines{
        public static final int AprilTag=0;
        public static final int TopTape=1;
        public static final int BottomTape=2;
        public static final int AprilTagID1=3;
        public static final int AprilTagID6=4;
    }

    public static enum TargetHeight
    {
                    TOP,MIDDLE,BOTTOM;	
    }
    
    public class IntakeConvey{
        public static final int motorPort=42;
        public static final double inSpeed=-0.8;
        public static final double outSpeed=0.8;
    }
    public class IntakeArms{
        public static final int masterMotorPort=13;
        public static final int slaveMotorPort=16; // FIX
        public static final double homeSpeed=0.10;        
        public static final double homeTimeOut=2.0;
        public static final double inSpeed=0.3;
        public static final double outSpeed=-0.3;
        public static final int limitSwitch1 = 1;
        public static final double posHumanPlayer=6;
        public static final double posExtendLimit=8;
        public static final double autoMoveSpeed=.25;
        public static final double autoMoveTolerance=0.1;
    }
    public class Spindexer {
        public static final int motorPort =17;
        public static final double spinForward=0.9;
        public static final double spinBack=-0.9;
        public static final double spinForwardSlow=0.6;
        public static final double spinBackSlow=-0.6;
    }

    public class Paddle{
        public static final int intakePort=33;
        public static final double intakeInSpeed=-0.50;
        public static final double intakeOutSpeed=0.50;
        public static final int limitSwitch=2;
        public static final double homePaddleSpeed=-0.1;
        public static final double homeTimeOut=1;
        public static final double encoderMoveOutPosStart=-2;
        public static final double encoderMoveOutPosEnd=2;
    
    }    
    public class Elevator{
        public static final int elevatorPort=19;
        public static final double elevatorSpeed=.50;
        public static final double elevatorLimitSpeed=.50;
    }
    public class HungryHippoValues{
        public static final int armPos1=10;
        public static final int armPos2=10;
        public static final int armPos3=10;
        public static final int armPos4=10;
        public static final int armPos5=10;
        public static final int armPos6=10;
        public static final int armPos7=10;
        public static final int armPos8=10;
        public static final double armSpeed1=-0.10;
        public static final double armSpeed2=-0.10;
        public static final double armSpeed3=-0.10;
        public static final double armSpeed4=-0.10;
        public static final double armSpeed5=-0.10;
        public static final double armSpeed6=-0.10;
        public static final double armSpeed7=-0.10;
        public static final double armSpeed8=-0.10;
        public static final int paddlePos1=20;
        public static final int paddlePos2=20;
        public static final int paddlePos3=20;
        public static final int paddlePos4=20;
        public static final int paddlePos5=20;
        public static final int paddlePos6=20;
        public static final int paddlePos7=20;
        public static final int paddlePos8=20;
        public static final double paddleSpeed1=0.10;
        public static final double paddleSpeed2=0.10;
        public static final double paddleSpeed3=0.10;
        public static final double paddleSpeed4=0.10;
        public static final double paddleSpeed5=0.10;
        public static final double paddleSpeed6=0.10;
        public static final double paddleSpeed7=0.10;
        public static final double paddleSpeed8=0.10;
        
    }
    public class RotateArm{
        public static final int rotateArmPort=50;
        public static final int absoluteEncoder=0;
        public static final double manualSpeed=0.4;
        public static final double manualLimitSpeed=0.4;
        public static final double encodervalueStartPos=.538;
        public static final double encoderSpindexerHitLimit=.618;
        
    }
    public class swerveDrive{
        public static final double driveDeadband=0.05;
        public static final int kFrontLeftSteering = 1;
        public static final int kFrontRightSteering = 3;
        public static final int kBackLeftSteering = 2;
        public static final int kBackRightSteering = 0;
        public static final int kFrontLeftDrive = 7;
        public static final int kFrontRightDrive = 4;
        public static final int kBackLeftDrive = 5;
        public static final int kBackRightDrive = 6;
    }

    public static final class Lift{
        public static final double liftAutoExtendStage1Speed=-0.80;
        public static final double liftAutoExtendStage2Speed=-0.4;
        public static final double liftAutoRetractSpeed=0.3;
        public static final double liftAutoRetractHomeSpeed=0.1;
        public static final double posInitLift=-20;
        public static final double posLowFullLiftStage1=-100.00;
        public static final double posLowFullLiftStage2=-111.50;    
        public static final double posMiddleFullLiftStage1=-99.00;
        public static final double posMiddleFullLiftStage2=-110.00;    
        public static final double posHighFullLiftStage1=-99.00;
        public static final double posHighFullLiftStage2=-111.50;    
        public static final double posHome=10;
    }

    public static final class Rotate{
        public static final double rotateAutoOutStage1Speed=-0.75;
        public static final double rotateAutoOutStage2Speed=-0.4; 
        public static final double rotateAutoInSpeed=0.5; 
        public static final double angleLowConeStage1=.28;    
        public static final double angleLowConeStage2=.30;  
        public static final double angleMiddleConeStage1=.15;    
        public static final double angleMiddleConeStage2=.18;  
        public static final double angleHighConeStage1=.15;    
        public static final double angleHighConeStage2=.18;  
        public static final double angleIntakePos=0.07;
        public static final double homePos = 0.075;
        public static final double homeTimeFailsafe=5;
        public static final double homeSpeedForward=0.03;
        public static final double homeSpeedBackward=-0.03;

    }

    public static final class Pneumatics {
        public static final int CompressorID=0; 
        public static final int HubID=62;
        public static final PneumaticsModuleType moduleType=PneumaticsModuleType.REVPH;
    
        public static final int clawSolenoid = 8;
             
    }
    public static final class ButtonConstants{
        public static final int DriverIntakeOut=2;
        public static final int DriverIntakeIn=3;
        public static final int DriverPipelineLowTape=5;
        public static final int DriverPipelineHighTape=6;
        public static final int DriverDriveMode=7;
        public static final int DriverGyroReset=8;
  
        public static final int OperatorAutoLow=11;
        public static final int OperatorArmReturn=12;
        public static final int OperatorAutoBalance=13;
        public static final int OperatorAutoMiddle=14;
        
        public static final int OperatorIntakeIn=1;
        public static final int OperatorIntakeOut=2;
        public static final int OperatorClawSwap=3;
        public static final int OperatorPlayerStation=4;
        public static final int OperatorClawClose=5;
        public static final int OperatorClawOpen=6;
     
        public static final int OperatorSpindexPOVSL=0;
        public static final int OperatorSpindexPOVSR=180;
        public static final int OperatorSpindexPOVFL=270;
        public static final int OperatorSpindexPOVFR=90;
       

        public static final int TargetTopLeft=4;
        public static final int TargetMiddleLeft=5;
        public static final int TargetBottomLeft=6;
        public static final int TargetTopCenter=7;
        public static final int TargetMiddleCenter=8;
        public static final int TargetBottomCenter=9;
        public static final int TargetTopRight=10;
        public static final int TargetMiddleRight=11;
        public static final int TargetBottomRight=12;

        public static final double JoystickDeadBand = 0.10;
        public static final double ElevatorDeadBand = 0.10;
        public static final double RotateDeadBand = 0.10;
        public static final double LeftTriggerDeadBand = 0.05;
        public static final double RightTriggerDeadBand = 0.05;
    }

    public static final class LimitSwitches{
        public static final int ExampleSwitch=0;
    }

    public static final class AutoModes {
        public static final String autoMode0="0-Do Nothing";
        public static final String autoMode1="1-Push Forward/Move Back Out of Zone";
        public static final String autoMode2="1=Score Cube / Move out of Zone";
        public static final String autoMode3="1=Score Cone / Move out of Zone";
        public static final String autoMode4="1-Score Cube / Dock Charging Station";
        public static final String autoMode5="1-Score Cone / Dock Charging Station";
        public static final String autoMode6="1-Score Cube / Engage Charging Station";
        public static final String autoMode7="1-Score Cone / Engage Charging Station";
        public static final String autoMode8="2-Score Cone / Leave / Get Cone / Score";
        public static final String autoMode9="2-Score Cube / Leave / Get Cone / Score";

       
        public static final int autoNothing = 0;
        public static final int autoMoveBack = 1;
        public static final int autoCubeLeave = 2;
        public static final int autoConeLeave = 3;
        public static final int autoCubeDock = 4;
        public static final int autoConeDock = 5;
        public static final int autoCubeEngage = 6;
        public static final int autoConeEngage = 7;
        public static final int autoConeScore2=8;
        public static final int autoCubeScore2=9;
        
        
        
        public static final String delayMode0="0 Seconds";
        public static final String delayMode1="3 Seconds";
        public static final String delayMode2="5 Seconds";
        public static final String delayMode3="8 Seconds";
        public static final int delayValMode0=0;
        public static final int delayValMode1=3;
        public static final int delayValMode2=5;
        public static final int delayValMode3=8;
        public static final int defaultDelayMode=0;
        public static final double MoveSpeed=0.5;
        public static final double LeaveCommunityDistance=174; // 15 feet = 174-30
        public static final double pushDistance = 5;
        public static final double DistanceToCharging=20;// x feet
        public static final double DistanceToDock=30;
    }
            
        
    public static final class InputControllers {
        public static final int kXboxDrive = 0;
        public static final int kXboxOperator = 1;
        public static final int kCustomController = 1;
    }
    public static final class PhotonVision{
        public static String camera="";
    }

    /*
     * Constant Values for Limelight based on target and mounting
     */
    public static final class LimeLightValues {
    
        public static final double steeringP = 0.035;
        public static final double steeringI = 0;
        public static final double steeringD = 0.0055;
        public static final double steeringFeedForward = 0.0;

        public static final double targetHeight = 18; // 249 cm
        public static final double targetXPosShoot = -1.5;
        public static final double targetXPosSafeZone = 5;
        public static final double targetXPosRange=50;
        public static final double limelightHeight = 19; //37
        public static final double limelightAngle = 0; //40
        public static final double kVisionDistanceTolerance = 5;
        public static final double kVisionXTolerance = 1;
        public static final double kVisionXOffset=4;
        public static final double kVisionXMaxDistanceOffset=4.31; // was 1.7
        public static final double kVisionXMinDistanceOffset=0.91; // was 1.7
    }

    public class LEDS {
        public static final int PORT = 0;
        public static final int COUNT = 256;
        public static final int FLASH_DELAY=5;
        

        public class Colors {
            public static final int RED = 0;
            public static final int PINK = 1;
            public static final int PURPLE = 2;
            public static final int BLUE = 3;
            public static final int CYAN = 4;
            public static final int GREEN = 5;
            public static final int YELLOW = 6;
                public static final int ORANGE = 7;
        }
    }
       
    public static class RobotMap {
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 20; // CAN
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 2;//1;; // Analog
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 25; // CAN
    
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 21; // CAN
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER =1;//2; // Analog
        public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 26; // CAN
    
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 27; // CAN
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 0; // Analog
        public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 22; // CAN
    
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 28; // CAN
        public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 3; // Analog
        public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 23; // CAN
           }
    
   public static final class PIDSteering{
    public static final double rightKP=-0.09;
    public static final double leftKP=0.09;
    public static final double rightKI=-0;  
    public static final double leftKI=0;
    public static final double rightKD=-0;
    public static final double leftKD=0;
    public static final double forwardKP= -0.45;
    public static final double backwardKP = 0.45;
    public static final double forwardKI = -0;
    public static final double backwardKI = 0;
    public static final double forwardKD = -0;
    public static final double backwardKD = 0;

}
// AdvantageKit Constants
public static final Mode currentMode = Mode.REAL;

public static enum Mode {
  /** Running on a real robot. */
  REAL,

  /** Running a physics simulator. */
  SIM,

  /** Replaying from a log file. */
  REPLAY
}
}