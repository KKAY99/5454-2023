// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.DriveControlMode;
import frc.robot.classes.LEDStrip;
import frc.robot.classes.Limelight;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.awt.Color;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private NavX m_NavX = new NavX(SPI.Port.kMXP);
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autonomous Program");
    private final LoggedDashboardChooser<Command> autoDelay = new LoggedDashboardChooser<>("Auto Delay");
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
    private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
    private final PnuematicsSubystem m_PnuematicsSubystem = new PnuematicsSubystem(Constants.Pneumatics.HubID,
                                                                   Constants.Pneumatics.moduleType,
                                                                   Constants.Pneumatics.clawSolenoid,
                                                                   Constants.Pneumatics.punchSolenoid);
    private final IntakeArmsSubsystem m_IntakeArms = new IntakeArmsSubsystem(Constants.IntakeArms.masterMotorPort,Constants.IntakeArms.slaveMotorPort,
                                                 Constants.IntakeArms.limitSwitch1,Constants.IntakeArms.posExtendLimit);
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight,
                                                 Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset,80);
    private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem(Constants.Elevator.elevatorPort, Constants.Elevator.limitSwitch);
    private final RotateArmSubsystem m_Rotate = new RotateArmSubsystem(Constants.RotateArm.rotateArmPort,
                                                       Constants.RotateArm.absoluteEncoder,
                                                       Constants.RotateArm.encodervalueHomePos,
                                                       Constants.RotateArm.encoderFrontLimit,
                                                       Constants.RotateArm.encoderBackLimit);
    private final IntakeConveySubsystem m_Convey = new IntakeConveySubsystem(Constants.IntakeConvey.motorPort);
    private final PaddleSubsystem m_paddle = new PaddleSubsystem(Constants.Paddle.intakePort,
                                                                  Constants.Paddle.limitSwitch,
                                                                  Constants.Paddle.homePaddleSpeed,
                                                                  Constants.Paddle.encoderMovePosStart,
                                                                  Constants.Paddle.encoderMovePosEnd);


     private final LEDStrip m_ledStrip = new LEDStrip(Constants.LEDS.PORT, Constants.LEDS.COUNT);
     private static enum LEDMode
     {
                     NOTSET,DISBLED, AUTOMODE, OFFTARGET, OFFTARGETSWEET, ONTARGETSWEET,ONTARGET,SHOOTING,CLIMBING,TELEOP;	
     }
     private LEDMode m_LEDMode=LEDMode.DISBLED;
     private boolean m_disabled=true;
     private boolean m_homed=false;
     private boolean m_ledFlash=false;
     private boolean m_ledFlashMode=false;
     private int m_ledFlashDelayCount=0;
     private static final int LEDMODE_WAVE = 0;
     private static final int LEDMODE_BAR = 1;
     private static final int LEDMODE_RAINBOW = 2;
     private static final int LEDMODE_SOLID = 3;
     private static final int LEDMODE_OFF = 4;
     private LEDMode m_oldLEDmode=LEDMode.NOTSET;  
   
    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
   
    static LoggedDashboardString dashDriveMode= new LoggedDashboardString("Drive Mode", "Field"); 
    
    static LoggedDashboardNumber networkTableEntryVisionDistance = new LoggedDashboardNumber("Vision Distance", 0);

  //  static LoggedDashboardNumber networkTableEntryFrontLeftSpeed = new LoggedDashboardNumber("FL Speed", 0);

  //  static LoggedDashboardNumber networkTableEntryFrontRightSpeed = new LoggedDashboardNumber("FR Speed", 0);

  //  static LoggedDashboardNumber networkTableEntryBackLeftSpeed = new LoggedDashboardNumber("BL Speed", 0);

  //  static LoggedDashboardNumber networkTableEntryBackRightSpeed = new LoggedDashboardNumber("BR Speed", 0);

 //   static LoggedDashboardNumber networkTableEntryFrontLeftEncoderActual = new LoggedDashboardNumber("FL Encoder Actual", 0);
 
 //   static LoggedDashboardNumber networkTableEntryFrontRightEncoderActual = new LoggedDashboardNumber("FR Encoder Actual", 0);

 //   static LoggedDashboardNumber networkTableEntryBackLeftEncoderActual = new LoggedDashboardNumber("BL Encoder Actual", 0);

 //   static LoggedDashboardNumber networkTableEntryBackRightEncoderActual = new LoggedDashboardNumber("BR Encoder Actual", 0);

//    static LoggedDashboardNumber networkTableEntryFrontLeftEncoderTarget = new LoggedDashboardNumber("FL Encoder Target", 0);

 //   static LoggedDashboardNumber networkTableEntryFrontRightEncoderTarget = new LoggedDashboardNumber("FR Encoder Target", 0);

  //  static LoggedDashboardNumber networkTableEntryBackLeftEncoderTarget = new LoggedDashboardNumber("BL Encoder Target", 0);

   // static LoggedDashboardNumber networkTableEntryBackRightEncoderTarget = new LoggedDashboardNumber("BR Encoder Target", 0);

    static LoggedDashboardNumber frontLeftAngle = new LoggedDashboardNumber("FL Angle", 0);

    static LoggedDashboardNumber frontRightAngle = new LoggedDashboardNumber("FR Angle", 0);

    static LoggedDashboardNumber backLeftAngle = new LoggedDashboardNumber("BL Angle", 0);

    static LoggedDashboardNumber backRightAngle = new LoggedDashboardNumber("BR Angle", 0);
  
    static LoggedDashboardString ShuffleboardLog = new LoggedDashboardString("ShuffleboardLog", "");

    static LoggedDashboardNumber shuffleboardGyroFused = new LoggedDashboardNumber("Gyro - Fused Heading", 0);

    static LoggedDashboardNumber gryoRoll = new LoggedDashboardNumber("Gyro Roll",0);

    static LoggedDashboardBoolean isOnTarget = new LoggedDashboardBoolean("Is On Target", false);
    
    static LoggedDashboardBoolean isTargetAvailable = new LoggedDashboardBoolean("Is Target Available", false);
    
    static LoggedDashboardBoolean isAtDistanceFromTarget = new LoggedDashboardBoolean("Is At Right Distance", false);
    
    static LoggedDashboardBoolean isAligned = new LoggedDashboardBoolean("Is Aligned With Target", false);

    static LoggedDashboardNumber elevatorEncoder = new LoggedDashboardNumber("Elevator Encoder", 0);

    static LoggedDashboardNumber rotateEncoder = new LoggedDashboardNumber("Rotate Encoder", 0);

    static LoggedDashboardNumber intakeArmsEncoder = new LoggedDashboardNumber("Intake Arms Encoder", 0);
 
    static LoggedDashboardNumber paddleEncoder = new LoggedDashboardNumber("Paddle Encoder", 0);

    static LoggedDashboardBoolean elevatorLimitSwitch = new LoggedDashboardBoolean("Elevator Limit Switch", false);

    static LoggedDashboardBoolean paddleLimitSwitch = new LoggedDashboardBoolean("Paddle Limit Switch", false);

    static LoggedDashboardBoolean intakeArmsLimitSwitch = new LoggedDashboardBoolean("Intake Arms Limit Switch", false);

    static LoggedDashboardBoolean rotateForwardSoftLimit = new LoggedDashboardBoolean("Rotate Forward Soft Limit", false);

    static LoggedDashboardBoolean rotateBackSoftLimit = new LoggedDashboardBoolean("Rotate Backward Soft Limit", false);

    static LoggedDashboardBoolean elevatorMaxLimit = new LoggedDashboardBoolean("Elevator Max Limit", false);

    static LoggedDashboardBoolean rotateHardLimit = new LoggedDashboardBoolean("Rotate Soft Limit", false);

    static LoggedDashboardNumber compressorPressure = new LoggedDashboardNumber("Compressor Pressure", 0);

    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);

    public RobotContainer() {
        // Configure the button bindings
        createAutoCommands();
        configureButtonBindings();

        m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));

    }

    private void createAutoCommands(){
    autoChooser.addDefaultOption(AutoModes.autoMode0, new AutoDoNothingCommand());
    Command commandAutoMoveBack= new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,0,AutoModes.pushDistance),
                                            new AutoMoveCommand(m_RobotDrive,180, AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode1,  commandAutoMoveBack);
    
    Command commandAutoConeLeave = new SequentialCommandGroup(new ClawCommand(m_PnuematicsSubystem, true, "auto"),
                                                              new zMoveArmExtend(m_Elevator, m_Rotate, Constants.TargetHeight.TOP),
                                                              new ClawCommand(m_PnuematicsSubystem, false, "auto"),
                                                              new zMoveArmRetract(m_Elevator,m_Rotate),
                                                              new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode2,  commandAutoConeLeave);
  
    
    Command commandAutoCubeLeave=  new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                            new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                            new ClawCommand(m_PnuematicsSubystem, false,"autoCube"),
                                            new zPivotArmResetCommand(),
                                            new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode3,commandAutoCubeLeave);
 
 Command commandAutoConeDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomConeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false,"autoCone"),
                                       new zPivotArmResetCommand(),
                                       new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
 autoChooser.addOption(AutoModes.autoMode4,commandAutoConeDock);
 
 Command commandAutoCubeDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomCubeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
 autoChooser.addOption(AutoModes.autoMode5, commandAutoCubeDock);
 
 Command commandAutoConeEngage  = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomConeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                               new zEngageonChargingCommand());

autoChooser.addOption(AutoModes.autoMode6,commandAutoConeEngage);
Command commandAutoCubeEngage = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomCubeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                               new zEngageonChargingCommand());

autoChooser.addOption(AutoModes.autoMode7,commandAutoCubeEngage);
 
Command commandAutoConeScore2= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, true,"auto"),
                                       new zPivotArmResetCommand(),
                                       new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cone),
                                       new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                       new zPivotArmResetCommand());
 autoChooser.addOption(AutoModes.autoMode8, commandAutoConeScore2);
 Command commandAutoCubeScore2 = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                       new zPivotArmResetCommand(),
                                       new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cube),
                                       new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                       new zPivotArmResetCommand());
                                       
autoChooser.addOption(AutoModes.autoMode9, commandAutoCubeScore2);
 

}

    
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //final PaddleCommand intakeInCommand = new PaddleCommand(m_paddle, Constants.Paddle.intakeInSpeed);
        //final PaddleCommand intakeOutCommand = new PaddleCommand(m_paddle, Constants.Paddle.intakeOutSpeed);
        final PaddleConveyCommand intakeInCommand = new PaddleConveyCommand(m_paddle,m_Convey,Constants.Paddle.intakeInSpeed,Constants.IntakeConvey.inSpeed);
        final PaddleConveyCommand intakeOutCommand = new PaddleConveyCommand(m_paddle, m_Convey,Constants.Paddle.intakeOutSpeed,-Constants.IntakeConvey.outSpeed);
        final GyroResetCommand gyroResetCommand = new GyroResetCommand(m_RobotDrive,m_Limelight);
        final SpindexerCommand spindexerLeftCommand = new SpindexerCommand(m_SpindexerSubsystem, Constants.Spindexer.spinBack);
        final SpindexerCommand spindexerRightCommand = new SpindexerCommand(m_SpindexerSubsystem, Constants.Spindexer.spinForward);
        final SpindexerCommand spindexerLeftSlowCommand = new SpindexerCommand(m_SpindexerSubsystem, Constants.Spindexer.spinBackSlow);
        final SpindexerCommand spindexerRightSlowCommand = new SpindexerCommand(m_SpindexerSubsystem, Constants.Spindexer.spinForwardSlow);
        
        final PipelineSwapCommand pipelineAprilTagCommand = new PipelineSwapCommand(m_Limelight, 0, Constants.ChargedUp.targetHeightAprilTag);
        final PipelineSwapCommand pipelineTapeLowCommand = new PipelineSwapCommand(m_Limelight, 1, Constants.ChargedUp.targetHeighMLowTape);
        final PipelineSwapCommand pipelineTapeHighCommand = new PipelineSwapCommand(m_Limelight, 2, Constants.ChargedUp.targetHeightHighTape);
        final IntakeArmsCommand ExtendArmsCommand = new IntakeArmsCommand(m_IntakeArms, Constants.IntakeArms.outSpeed);
        final IntakeArmsCommand RetractArmsCommand = new IntakeArmsCommand(m_IntakeArms, Constants.IntakeArms.inSpeed);
        final RotateCommand rotateCommand = new RotateCommand(m_Rotate,() -> (m_xBoxOperator.getLeftX()),Constants.RotateArm.manualLimitSpeed);
        final ElevatorCommand elevatorCommand = new ElevatorCommand(m_Elevator,() -> (m_xBoxOperator.getLeftY()), Constants.Elevator.elevatorLimitSpeed);
        final ClawSwapCommand swapClawCommand = new ClawSwapCommand(m_PnuematicsSubystem);
        final PaddleConveyRetractCommand intakeConveyandRetract = new PaddleConveyRetractCommand(Constants.IntakeArms.limitSwitch1,m_paddle, m_IntakeArms,m_Convey,
        Constants.Paddle.intakeInSpeed, Constants.IntakeArms.inSpeed, Constants.IntakeConvey.inSpeed);
        final SolenoidPunchCommand punchSolenoidCommand = new SolenoidPunchCommand(m_PnuematicsSubystem);
// Auto commands
        final zBalanceRobotCommand balanceRobotCommand = new zBalanceRobotCommand(m_NavX,m_RobotDrive);
        final zAutoTargetToColumnCommand targetColumnCommandUpperLeft = new zAutoTargetToColumnCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosUpperLeft);
        final zAutoTargetToColumnCommand targetColumnCommandUpperRight = new zAutoTargetToColumnCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosUpperRight);
        final zAutoTargetandMoveCommand tapetargetandMoveCommand = new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
        Constants.ChargedUp.GridPosMiddleLeft);
        final zAutoTargetandMoveCommand tagtargetandMoveCommand = new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
        Constants.ChargedUp.GridPosMiddleCenter);
        final zHungryHippoCommand hungryHippoCommand = new zHungryHippoCommand(m_paddle, m_IntakeArms, m_Convey);
        final zMoveArmRetract retractElevatorCommand = new zMoveArmRetract(m_Elevator, m_Rotate);
        // *** 9 Box targeting
        final SequentialCommandGroup zAutoTargetTL= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                              new ClawCommand(m_PnuematicsSubystem, false,"zAuto"));
        Trigger targetTopLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetTopLeft);
        targetTopLeft.toggleOnTrue(zAutoTargetTL);

        final SequentialCommandGroup zAutoTargetML= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                              new ClawCommand(m_PnuematicsSubystem, false,"zAutoTargetML"));
        Trigger targetMiddleLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleLeft);
        targetMiddleLeft.toggleOnTrue(zAutoTargetML);

        final SequentialCommandGroup zAutoTargetBL= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosBottomLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                              new ClawCommand(m_PnuematicsSubystem, false,"zAutoTargetBL"));
        Trigger targetMiddleBottom= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomLeft);
        targetMiddleBottom.toggleOnTrue(zAutoTargetBL);

        final SequentialCommandGroup zAutoTargetTC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                              new ClawCommand(m_PnuematicsSubystem, false,"zAutoTargetTC"));
        Trigger targetTopCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetTopCenter);
        targetTopLeft.toggleOnTrue(zAutoTargetTC);

        final SequentialCommandGroup zAutoTargetMC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                              new ClawCommand(m_PnuematicsSubystem, false,"zAuto"));
        Trigger targetMiddleCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleCenter);
        targetMiddleCenter.toggleOnTrue(zAutoTargetMC);

        final SequentialCommandGroup zAutoTargetBC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosBottomCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                              new ClawCommand(m_PnuematicsSubystem, false,"zAuto"));
        Trigger targetBottomCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomCenter);
        targetBottomCenter.toggleOnTrue(zAutoTargetBC);

        final SequentialCommandGroup zAutoTargetTR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosUpperRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                               new ClawCommand(m_PnuematicsSubystem, false,"zAuto"));

        Trigger targetTopRight =new JoystickButton(m_CustomController,ButtonConstants.TargetTopRight);
        targetTopRight.toggleOnTrue(zAutoTargetTR);

        final SequentialCommandGroup zAutoTargetMR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                               new ClawCommand(m_PnuematicsSubystem,false,"zAuto"));
        Trigger targetMiddleRight= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleRight);
        targetMiddleRight.toggleOnTrue(zAutoTargetMR);

        final SequentialCommandGroup zAutoTargetBR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosBottomRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                               new ClawCommand(m_PnuematicsSubystem, false,"zAuto"));
        Trigger targetBottomRight= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomRight);
        targetBottomRight.toggleOnTrue(zAutoTargetBR);

        //final LatchCommand latchCommand =new LatchCommand(m_Pnuematics);
        Trigger swapClaw = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorClawSwap);
        swapClaw.toggleOnTrue(swapClawCommand);

        final zMoveArmRetract RetractArmCommand = new zMoveArmRetract(m_Elevator, m_Rotate);
        JoystickButton Retract=new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorArmReturn);
        Retract.onTrue(RetractArmCommand);

        final SequentialCommandGroup paddleHumanPlayer = new SequentialCommandGroup(
                                                         new IntakeArmsMoveToCommand(m_IntakeArms,
                                                              Constants.IntakeArms.posHumanPlayer,
                                                              Constants.IntakeArms.autoMoveTolerance,
                                                              Constants.IntakeArms.autoMoveSpeed),
                                                         new PaddleMoveToCommand(m_paddle,
                                                              Constants.Paddle.encoderHumanPlayerPos,
                                                              Constants.Paddle.autoMoveTolerance,
                                                              Constants.Paddle.autoMoveOutSpeed));
        Trigger humanPlayer = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorPlayerStation);
      
        humanPlayer.onTrue(paddleHumanPlayer);

        final SequentialCommandGroup autoLowMoveArm =new SequentialCommandGroup(//new ClawCommand(m_PnuematicsSubystem, true),
                                                           new zMoveArmExtend(m_Elevator, m_Rotate, Constants.TargetHeight.BOTTOM),
                                                           new ClawCommand(m_PnuematicsSubystem, false,"zAuto"),
                                                           new zMoveArmRetract(m_Elevator,m_Rotate));

        JoystickButton moveArmLow = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorAutoLow);
        moveArmLow.toggleOnTrue(autoLowMoveArm);

        final SequentialCommandGroup autoMiddleMoveArm =new SequentialCommandGroup(//new ClawCommand(m_PnuematicsSubystem, true),
                                                           new zMoveArmExtend(m_Elevator, m_Rotate, Constants.TargetHeight.MIDDLE)
                                                           // ClawCommand(m_PnuematicsSubystem, false),
                                                           //new zMoveArmRetract(m_Elevator,m_Rotate)
                                                           );

        JoystickButton moveArmMiddle = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorAutoMiddle);
        moveArmMiddle.toggleOnTrue(autoMiddleMoveArm);

        final SequentialCommandGroup autoHighMoveArm =new SequentialCommandGroup(//new ClawCommand(m_PnuematicsSubystem, true),
                                                           new zMoveArmExtend(m_Elevator, m_Rotate, Constants.TargetHeight.TOP)
                                                           //new ClawCommand(m_PnuematicsSubystem, false,"zAuto"),
                                                           //new zMoveArmRetract(m_Elevator,m_Rotate)
                                                           );

        Trigger moveArmHigh = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        moveArmHigh.toggleOnTrue(autoHighMoveArm);

        final SwitchDriveModeCommand switchDriveCommand=new SwitchDriveModeCommand(m_DriveControlMode);  
        
        Trigger driverSpindexerRight=new POVButton(m_xBoxOperator,ButtonConstants.OperatorSpindexPOVFR);
        driverSpindexerRight.whileTrue(spindexerRightCommand);

        Trigger driverSpindexerLeft=new POVButton(m_xBoxOperator,ButtonConstants.OperatorSpindexPOVFL);
        driverSpindexerLeft.whileTrue(spindexerLeftCommand);
       
        Trigger driverSpindexerSlowRight=new POVButton(m_xBoxOperator,ButtonConstants.OperatorSpindexPOVSR);
        driverSpindexerSlowRight.whileTrue(spindexerRightSlowCommand);

        Trigger driverSpindexerSlowLeft=new POVButton(m_xBoxOperator,ButtonConstants.OperatorSpindexPOVSL);
        driverSpindexerSlowLeft.whileTrue(spindexerLeftSlowCommand);
        
        Trigger driverMode=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverDriveMode);
        driverMode.toggleOnTrue(switchDriveCommand);
        
        Trigger driverSolenoidPunch =  new JoystickButton(m_xBoxDriver, ButtonConstants.DriverSolenoidPunch);
        driverSolenoidPunch.toggleOnTrue(punchSolenoidCommand);

        Trigger operatorIntakeIn =  new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeIn);
        operatorIntakeIn.whileTrue(intakeInCommand);

        Trigger operatorIntakeOut =  new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorIntakeOut);
        operatorIntakeOut.whileTrue(intakeOutCommand);

        Trigger driverGyroReset = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverGyroReset);
        driverGyroReset.whileTrue(gyroResetCommand);
        //TODO: Replace with Constant
        //Trigger driverConveyRetract = new Trigger(() -> Math.abs(m_xBoxDriver.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        //driverConveyRetract.toggleOnTrue(intakeConveyandRetract);

        Trigger driverLowTape = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverPipelineLowTape);
        driverLowTape.toggleOnTrue(pipelineTapeLowCommand);

        Trigger driverHighTape = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverPipelineHighTape);
        driverHighTape.toggleOnTrue(pipelineTapeHighCommand);
        
        Trigger driverPipelineAprilTag = new Trigger(() -> Math.abs(m_xBoxDriver.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        driverPipelineAprilTag.toggleOnTrue(pipelineAprilTagCommand);
/*            
        Trigger operatorIntakeIn =  new JoystickButton(m_xBoxDriver, ButtonConstants.OperatorIntakeIn);
        operatorIntakeIn.whileTrue(intakeInCommand);

        Trigger operatorIntakeOut =  new JoystickButton(m_xBoxDriver, ButtonConstants.OperatorIntakeOut);
        operatorIntakeOut.whileTrue(intakeInCommand);

        Trigger operatorGyroReset = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorGyroReset);
        operatorGyroReset.whileTrue(gyroResetCommand);

 */       
        Trigger operatorAutoBalance = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorAutoBalance);
        operatorAutoBalance.toggleOnTrue(balanceRobotCommand);

        Trigger operatorElevator = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftY())>ButtonConstants.ElevatorDeadBand);
        operatorElevator.whileTrue(elevatorCommand);
        
        Trigger operatorRotate = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftX())>ButtonConstants.RotateDeadBand);
        operatorRotate.whileTrue(rotateCommand);

        Trigger operatorIntakeExtend = new Trigger(() -> (m_xBoxOperator.getRightX())>ButtonConstants.JoystickDeadBand);
        operatorIntakeExtend.whileTrue(ExtendArmsCommand);
        
        Trigger operatorIntakeRetract = new Trigger(() -> (m_xBoxOperator.getRightX())<0-ButtonConstants.JoystickDeadBand);
        operatorIntakeRetract.whileTrue(RetractArmsCommand);

        Trigger operatorTapeAlign = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(2))>ButtonConstants.LeftTriggerDeadBand);
        operatorTapeAlign.toggleOnTrue(tapetargetandMoveCommand);
        
        Trigger operatorTagAlign = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        operatorTagAlign.toggleOnTrue(tagtargetandMoveCommand);

        Trigger operatorHungryHippo = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorHungryHippo);
        operatorHungryHippo.toggleOnTrue(hungryHippoCommand);

        Trigger operatorAutoRetractElevator = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorRetractElevator);
        operatorAutoRetractElevator.toggleOnTrue(retractElevatorCommand);

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Integer selectedMode) {
        Command autoCommand = new AutoDoNothingCommand(); // Default Command is DoNothing
        //
         switch (selectedMode){
          case AutoModes.autoMoveBack:
             autoCommand= new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,0,AutoModes.pushDistance),
                                                     new AutoMoveCommand(m_RobotDrive,180, AutoModes.LeaveCommunityDistance));
            break;
          case AutoModes.autoConeLeave:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                          Constants.ChargedUp.GridPosBottomConeAny),
                                                      new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                      new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                      new zPivotArmResetCommand(),
                                                      new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
            break;
          case AutoModes.autoCubeLeave:
            autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                         Constants.ChargedUp.GridPosBottomCubeAny),
                                                     new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                     new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                     new zPivotArmResetCommand(),
                                                     new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
           break;
          case AutoModes.autoConeDock:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomConeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                new zPivotArmResetCommand(),
                                                new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
          break;
          case AutoModes.autoCubeDock:
            autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
          break;
          case AutoModes.autoConeEngage:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomConeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                                        new zEngageonChargingCommand());
         
          break;            
          case AutoModes.autoCubeEngage:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                                        new zEngageonChargingCommand());

          break;
          case AutoModes.autoConeScore2:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                new zPivotArmResetCommand(),
                                                new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cone),
                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                new zPivotArmResetCommand());
         break;
          case AutoModes.autoCubeScore2:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                new zPivotArmResetCommand(),
                                                new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cube),
                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                new zPivotArmResetCommand());
                                                
          default:
            autoCommand = new AutoDoNothingCommand();
        }
        return autoCommand;
        // return m_autoCommand;
    }   
    public void refreshSmartDashboard()
    {  
        frontLeftAngle.set(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.set(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.set(m_RobotDrive.getBackLeftAngle());
        backRightAngle.set(m_RobotDrive.getbackRightAngle());
        elevatorEncoder.set(m_Elevator.getElevatorPos());
        rotateEncoder.set(m_Rotate.getRelativeRotatePos());
        intakeArmsEncoder.set(m_IntakeArms.getPos());
        paddleEncoder.set(m_paddle.getPos());
        elevatorLimitSwitch.set(m_Elevator.hasHitPhysicalLimitSwitch());
        intakeArmsLimitSwitch.set(m_IntakeArms.hitPhysicalLimitSwitch());
        paddleLimitSwitch.set(m_paddle.hitPhysicalLimitSwitch());
        rotateBackSoftLimit.set(m_Rotate.hasHitBackSoftLimit());
        rotateForwardSoftLimit.set(m_Rotate.hasHitForwardSoftLimit());
        elevatorMaxLimit.set(m_Elevator.hasHitMaxLimit());
        gryoRoll.set(m_NavX.getAxis(Axis.ROLL));
        compressorPressure.set(m_PnuematicsSubystem.getPressure());

        SmartDashboard.putNumber("Rotate ABS", m_Rotate.getAbsolutePos());

        m_Limelight.update();
        if(m_RobotDrive.isFieldCentric()){
                dashDriveMode.set("Field");
        }else{
                dashDriveMode.set("Robot");
        }

        if(m_Limelight.isTargetAvailible()){
                isTargetAvailable.set(true);
        }else{
                isTargetAvailable.set(false);
        }

        if(Math.abs(m_Limelight.getXRaw()) <= Constants.LimeLightValues.kVisionXTolerance){
                isAligned.set(true);
        }else{
                isAligned.set(false);
        }

        //override disabled led mode
        if(m_disabled){
                m_LEDMode=LEDMode.DISBLED;
        }
        LEDUpdate();
     
}
    
     

     public void disabledPerioidicUpdates(){
        LEDUpdate();
     

    }

    public void disableLimelights(){
            m_Limelight.turnLEDOff();
    }
    public void enableLimelights(){
            m_Limelight.turnLEDOn();
            m_disabled=false;
    }
   
    public void resetDriveModes(){
       m_RobotDrive.resetDriveMode();
    }
   
    private void LEDUpdate(){            
        if(m_LEDMode!=m_oldLEDmode){            
                if(m_LEDMode==LEDMode.ONTARGET){
                        m_ledStrip.setColor(Colors.YELLOW);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.ONTARGETSWEET){
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=true;
                }
                if(m_LEDMode==LEDMode.OFFTARGETSWEET){
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.OFFTARGET){
                        m_ledStrip.setColor(Colors.RED);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.SHOOTING){
                        m_ledStrip.setColor(Colors.BLUE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.CLIMBING){
                        m_ledStrip.setColor(Colors.ORANGE);
                        m_ledStrip.setMode(LEDMODE_SOLID);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.AUTOMODE){
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledStrip.setMode(LEDMODE_RAINBOW);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.DISBLED){
                        m_ledStrip.setMode(LEDMODE_WAVE);
                        m_ledStrip.setColor(Colors.PURPLE);
                        m_ledFlash=false;
                }
                if(m_LEDMode==LEDMode.TELEOP){
                        m_ledStrip.setMode(LEDMODE_WAVE);
                        m_ledStrip.setColor(Colors.PINK);
                        m_ledFlash=false;
                }
        }
        if(m_ledFlash){
                m_ledFlashDelayCount++;
                if(m_ledFlashMode==false){
                        //(m_ledFlashDelayCouunt>10)
                        if(m_ledFlashDelayCount>Constants.LEDS.FLASH_DELAY) {
                                m_ledStrip.setMode(LEDMODE_OFF);
                                m_ledFlashMode=true;
                                m_ledStrip.update();
                                m_ledFlashDelayCount=0;
                        }
                } else{
                        if(m_ledFlashDelayCount>Constants.LEDS.FLASH_DELAY) {                       
                                m_ledStrip.setMode(LEDMODE_SOLID);                        
                                m_ledFlashMode=false;
                                m_ledStrip.update();
                                m_ledFlashDelayCount=0;
                        }
                }

        } else {
                m_ledStrip.update();
       
        }
        m_oldLEDmode=m_LEDMode;
            
    }
  
  
        public void AutoMode(){
                m_LEDMode=LEDMode.AUTOMODE;  
                LEDUpdate();
                homeRobot();
                //Set Default Pipeline to AprilTags
                m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
                
        }  
        public void TeleopMode(){
                m_LEDMode=LEDMode.TELEOP;  
                LEDUpdate();
                homeRobot();
                //Set Default Pipeline to AprilTags
                m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
                
        }
   
    private void homeRobot(){
        
        if(m_homed==false){
           homeIntakeArms();
                  homePaddle();
                  homeRotate();
                  homeElevator();
                //TODO: ROTATE home rotate wheel
                m_homed=true;
        }
    }
    private void homeRotate(){
        //System.out.println("start homing rotate");
        if(m_Rotate.hasHomed()==false){
                ClawCommand homeCloseClawCommand = new ClawCommand(m_PnuematicsSubystem, true,"rotateHome");
                CommandScheduler.getInstance().schedule(homeCloseClawCommand);
                zHomeRotateCommand resetRotate = new zHomeRotateCommand(m_Rotate,Constants.RotateArm.encodervalueHomePos, Constants.Rotate.homeSpeedForward,Constants.Rotate.homeTimeFailsafe) ; 
                CommandScheduler.getInstance().schedule(resetRotate);
        }
    }private void homePaddle(){
        System.out.println("start homing paddle");
        if(m_paddle.hasHomed()==false){
                zHomePaddleCommand resetPaddle = new zHomePaddleCommand(m_paddle,Constants.Paddle.homePaddleSpeed,Constants.Paddle.homeTimeOut) ; 
                CommandScheduler.getInstance().schedule(resetPaddle);
        }
    }
    private void homeElevator(){
        //System.out.println("start homing paddle");
        if(m_paddle.hasHomed()==false){
                zHomeElevatorCommand resetElevator = new zHomeElevatorCommand(m_Elevator,Constants.Elevator.elevatorLimitSpeed,Constants.Elevator.homeTimeOut) ; 
                CommandScheduler.getInstance().schedule(resetElevator);
        }
    }
    private void homeIntakeArms(){
        if(m_IntakeArms.hasHomed()==false){
                zHomeIntakeArmsCommand resetArms = new zHomeIntakeArmsCommand(m_IntakeArms,Constants.IntakeArms.homeSpeed,Constants.IntakeArms.homeTimeOut); 
                CommandScheduler.getInstance().schedule(resetArms);
            }
        }
    
    public void DisableMode(){
            m_disabled=true;
            m_LEDMode=LEDMode.DISBLED;
            LEDUpdate();
    }
    public void EnableMode(){
      m_disabled=false;
}

   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return autoChooser.get();
      }
}    
