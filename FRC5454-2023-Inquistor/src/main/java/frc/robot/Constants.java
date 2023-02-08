 
package frc.robot;

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
    
    public class Intake{
        public static final int intakePort=90;
        public static final double intakeInSpeed=0.50;
        public static final double intakeOutSpeed=-0.50;
    }    
    public class Elevator{
        public static final int elevatorPort=91;
        public static final double elevatorSpeed=.50;

    }
    public class Claw{
        public static final int clawPort=92;
        public static final double GrabSpeed=0.5;
        public static final double ReleaseSpeed=0.5;
    }
    public class RotateArm{
        public static final int rotateArmPort=93;
        
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

    public static final class Pneumatics {
        public static final int CompressorID=0; 
             
    }
    public static final class ButtonConstants{
        public static final int DriverDriveMode=1;
        public static final int DriverIntakeIn=2;
        public static final int DriverIntakeOut=3;
        public static final int DriverBalance=4;
        public static final int DriverGyroReset=7;
  
        public static final int OperatorIntakeIn=6;
        public static final int OperatorIntakeOut=7;
     
        public static final int OperatorGyroReset=7;
     
        public static final int TargetTopLeft=4;
        public static final int TargetMiddleLeft=5;
        public static final int TargetBottomLeft=6;
        public static final int TargetTopCenter=7;
        public static final int TargetMiddleCenter=8;
        public static final int TargetBottomCenter=9;
        public static final int TargetTopRight=10;
        public static final int TargetMiddleRight=11;
        public static final int TargetBottomRight=12;
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
        public static final double kVisionXTolerance = .1;
        public static final double kVisionXOffset=4;
        public static final double kVisionXMaxDistanceOffset=4.31; // was 1.7
        public static final double kVisionXMinDistanceOffset=0.91; // was 1.7
    }

    public class LEDS {
        public static final int PORT = 0;
        public static final int COUNT = 215;
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
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 25; // CAN
        public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; // Analog
        public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 20; // CAN
    
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 26; // CAN
        public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 1; // Analog
        public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 21; // CAN
    
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 27; // CAN
        public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 2; // Analog
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

