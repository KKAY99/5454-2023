// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoModes;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.InputControllers;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.DriveControlMode;
import frc.robot.classes.LEDStrip;
import frc.robot.classes.Limelight;
import frc.robot.commands.AutoDoNothingCommand;
import frc.robot.commands.AutoMoveCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.GyroResetCommand;
import frc.robot.commands.IntakeArmsCommand;
import frc.robot.commands.PaddleCommand;
import frc.robot.commands.PaddleConveyRetractCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.SpindexerCommand;
import frc.robot.commands.SwitchDriveModeCommand;
import frc.robot.commands.zAutoDetectandGetCommand;
import frc.robot.commands.zAutoTargetToColumnCommand;
import frc.robot.commands.zAutoTargetandMoveCommand;
import frc.robot.commands.zBalanceRobotCommand;
import frc.robot.commands.zEngageonChargingCommand;
import frc.robot.commands.zPivotArmResetCommand;
import frc.robot.commands.zPivotandExtendCommand;
import frc.robot.commands.zEngageonChargingCommand;
import frc.robot.commands.zMoveArmExtend;
import frc.robot.commands.zMoveArmRetract;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.subsystems.*;

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
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(null);
    private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
    private final PnuematicsSubystem m_PnuematicsSubystem = new PnuematicsSubystem(Constants.Pneumatics.nodeID,Constants.Pneumatics.moduleType,Constants.Pneumatics.clawSolenoid);
    private final IntakeArmsSubsystem m_IntakeArms = new IntakeArmsSubsystem(null, null, null, null);
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset,80);
    private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem(null);
    private final RotateArmSubsystem m_Rotate = new RotateArmSubsystem(0);
    private final IntakeConveySubsystem m_Convey = new IntakeConveySubsystem(null);


     private final LEDStrip m_ledStrip = new LEDStrip(Constants.LEDS.PORT, Constants.LEDS.COUNT);
     private static enum LEDMode
     {
                     NOTSET,DISBLED, AUTOMODE, OFFTARGET, OFFTARGETSWEET, ONTARGETSWEET,ONTARGET,SHOOTING,CLIMBING,TELEOP;	
     }
     private LEDMode m_LEDMode=LEDMode.DISBLED;
     private boolean m_disabled=true;
     private boolean m_ledFlash=false;
     private boolean m_ledFlashMode=false;
     private int m_ledFlashDelayCount=0;
     private static final int LEDMODE_WAVE = 0;
     private static final int LEDMODE_BAR = 1;
     private static final int LEDMODE_RAINBOW = 2;
     private static final int LEDMODE_SOLID = 3;
     private static final int LEDMODE_OFF = 4;
     private LEDMode m_oldLEDmode=LEDMode.NOTSET;  
     private final PaddleSubsystem m_paddle = new PaddleSubsystem(Constants.Paddle.intakePort);
   
    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
   
    static LoggedDashboardString dashDriveMode= new LoggedDashboardString("Drive Mode", "Field"); 
    
    static LoggedDashboardNumber networkTableEntryVisionDistance = new LoggedDashboardNumber("Vision Distance", 0);

    static LoggedDashboardNumber networkTableEntryFrontLeftSpeed = new LoggedDashboardNumber("FL Speed", 0);

    static LoggedDashboardNumber networkTableEntryFrontRightSpeed = new LoggedDashboardNumber("FR Speed", 0);

    static LoggedDashboardNumber networkTableEntryBackLeftSpeed = new LoggedDashboardNumber("BL Speed", 0);

    static LoggedDashboardNumber networkTableEntryBackRightSpeed = new LoggedDashboardNumber("BR Speed", 0);

    static LoggedDashboardNumber networkTableEntryFrontLeftEncoderActual = new LoggedDashboardNumber("FL Encoder Actual", 0);
 
    static LoggedDashboardNumber networkTableEntryFrontRightEncoderActual = new LoggedDashboardNumber("FR Encoder Actual", 0);

    static LoggedDashboardNumber networkTableEntryBackLeftEncoderActual = new LoggedDashboardNumber("BL Encoder Actual", 0);

    static LoggedDashboardNumber networkTableEntryBackRightEncoderActual = new LoggedDashboardNumber("BR Encoder Actual", 0);

    static LoggedDashboardNumber networkTableEntryFrontLeftEncoderTarget = new LoggedDashboardNumber("FL Encoder Target", 0);

    static LoggedDashboardNumber networkTableEntryFrontRightEncoderTarget = new LoggedDashboardNumber("FR Encoder Target", 0);

    static LoggedDashboardNumber networkTableEntryBackLeftEncoderTarget = new LoggedDashboardNumber("BL Encoder Target", 0);

    static LoggedDashboardNumber networkTableEntryBackRightEncoderTarget = new LoggedDashboardNumber("BR Encoder Target", 0);

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
 
    private XboxController m_xBoxDriver = new XboxController(InputControllers.kXboxDrive);
    private XboxController m_xBoxOperator = new XboxController(InputControllers.kXboxOperator);
    private Joystick m_CustomController = new Joystick(InputControllers.kCustomController);

    public RobotContainer() {
        // Configure the button bindings
        createAutoCommands();
        configureButtonBindings();

        m_RobotDrive.setDefaultCommand(
                new DefaultDriveCommand(m_RobotDrive,
                        () -> -m_xBoxDriver.getRightX(),
                        () -> m_xBoxDriver.getLeftY(),
                        () -> m_xBoxDriver.getLeftX(),
                        () -> m_DriveControlMode.isFieldOrientated()));

    }

    private void createAutoCommands(){
    autoChooser.addDefaultOption(AutoModes.autoMode0, new AutoDoNothingCommand());
    Command commandAutoMoveBack= new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,0,AutoModes.pushDistance),
                                            new AutoMoveCommand(m_RobotDrive,180, AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode1,  commandAutoMoveBack);
    Command commandAutoConeLeave = new SequentialCommandGroup(/*new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                 Constants.ChargedUp.GridPosBottomConeAny),
                                             new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                             new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                             new zPivotArmResetCommand(),*/
                                             new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode2,  commandAutoConeLeave);
  
    
    Command commandAutoCubeLeave=  new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                            new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                            new ClawCommand(m_PnuematicsSubystem, false),
                                            new zPivotArmResetCommand(),
                                            new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode3,commandAutoCubeLeave);
 
 Command commandAutoConeDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomConeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false),
                                       new zPivotArmResetCommand(),
                                       new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
 autoChooser.addOption(AutoModes.autoMode4,commandAutoConeDock);
 
 Command commandAutoCubeDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomCubeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_PnuematicsSubystem, false),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
 autoChooser.addOption(AutoModes.autoMode5, commandAutoCubeDock);
 
 Command commandAutoConeEngage  = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomConeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_PnuematicsSubystem, false),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                               new zEngageonChargingCommand());

autoChooser.addOption(AutoModes.autoMode6,commandAutoConeEngage);
Command commandAutoCubeEngage = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomCubeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_PnuematicsSubystem, false),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                               new zEngageonChargingCommand());

autoChooser.addOption(AutoModes.autoMode7,commandAutoCubeEngage);
 
Command commandAutoConeScore2= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, true),
                                       new zPivotArmResetCommand(),
                                       new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cone),
                                       new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false),
                                       new zPivotArmResetCommand());
 autoChooser.addOption(AutoModes.autoMode8, commandAutoConeScore2);
 Command commandAutoCubeScore2 = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false),
                                       new zPivotArmResetCommand(),
                                       new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cube),
                                       new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_PnuematicsSubystem, false),
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
        final PaddleCommand intakeInCommand = new PaddleCommand(m_paddle, Constants.Paddle.intakeInSpeed);
        final PaddleCommand intakeOutCommand = new PaddleCommand(m_paddle, Constants.Paddle.intakeOutSpeed);
        final GyroResetCommand gyroResetCommand = new GyroResetCommand(m_RobotDrive,m_Limelight);
        final SpindexerCommand spindexerLeftCommand = new SpindexerCommand(m_SpindexerSubsystem, 0.1);
        final SpindexerCommand spindexerRightCommand = new SpindexerCommand(m_SpindexerSubsystem, -0.1);
        final IntakeArmsCommand ExtendArmsCommand = new IntakeArmsCommand(m_IntakeArms, Constants.IntakeArms.limitSwitch,0.1);
        final IntakeArmsCommand RetractArmsCommand = new IntakeArmsCommand(m_IntakeArms, Constants.IntakeArms.limitSwitch, -0.1);
        final ClawCommand closeClawCommand = new ClawCommand(m_PnuematicsSubystem, m_disabled);
        final RotateCommand rotateCommand = new RotateCommand(m_Rotate,() -> (m_xBoxOperator.getRightX()),Constants.RotateArm.manualLimitSpeed);
        final ElevatorCommand elevatorCommand = new ElevatorCommand(m_Elevator,() -> (m_xBoxOperator.getLeftY()), Constants.Elevator.elevatorLimitSpeed);
        final ClawCommand openClawCommand = new ClawCommand(m_PnuematicsSubystem, m_disabled);
        final PaddleConveyRetractCommand intakeConveyandRetract = new PaddleConveyRetractCommand(Constants.IntakeArms.limitSwitch,m_paddle, m_IntakeArms,m_Convey,
        Constants.Paddle.intakeInSpeed, Constants.IntakeArms.inSpeed, Constants.IntakeConvey.inSpeed);
// Auto commands
        final zBalanceRobotCommand balanceRobotCommand = new zBalanceRobotCommand(m_NavX,m_RobotDrive);
        final zAutoTargetToColumnCommand targetColumnCommandUpperLeft = new zAutoTargetToColumnCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosUpperLeft);
        final zAutoTargetToColumnCommand targetColumnCommandUpperRight = new zAutoTargetToColumnCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosUpperRight);
        final zAutoTargetandMoveCommand tapetargetandMoveCommand = new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
        Constants.ChargedUp.GridPosMiddleLeft);
        final zAutoTargetandMoveCommand tagtargetandMoveCommand = new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
        Constants.ChargedUp.GridPosMiddleCenter);

        // *** 9 Box targeting
        final SequentialCommandGroup zAutoTargetTL= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                              new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetTopLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetTopLeft);
        targetTopLeft.toggleOnTrue(zAutoTargetTL);

        final SequentialCommandGroup zAutoTargetML= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                              new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetMiddleLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleLeft);
        targetMiddleLeft.toggleOnTrue(zAutoTargetML);

        final SequentialCommandGroup zAutoTargetBL= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosBottomLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                              new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetMiddleBottom= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomLeft);
        targetMiddleBottom.toggleOnTrue(zAutoTargetBL);

        final SequentialCommandGroup zAutoTargetTC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                              new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetTopCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetTopCenter);
        targetTopLeft.toggleOnTrue(zAutoTargetTC);

        final SequentialCommandGroup zAutoTargetMC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                              new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetMiddleCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleCenter);
        targetMiddleCenter.toggleOnTrue(zAutoTargetMC);

        final SequentialCommandGroup zAutoTargetBC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosBottomCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                              new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetBottomCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomCenter);
        targetBottomCenter.toggleOnTrue(zAutoTargetBC);

        final SequentialCommandGroup zAutoTargetTR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosUpperRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                               new ClawCommand(m_PnuematicsSubystem, false));

        Trigger targetTopRight =new JoystickButton(m_CustomController,ButtonConstants.TargetTopRight);
        targetTopRight.toggleOnTrue(zAutoTargetTR);

        final SequentialCommandGroup zAutoTargetMR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                               new ClawCommand(m_PnuematicsSubystem,false));
        Trigger targetMiddleRight= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleRight);
        targetMiddleRight.toggleOnTrue(zAutoTargetMR);

        final SequentialCommandGroup zAutoTargetBR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosBottomRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                               new ClawCommand(m_PnuematicsSubystem, false));
        Trigger targetBottomRight= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomRight);
        targetBottomRight.toggleOnTrue(zAutoTargetBR);

        //final LatchCommand latchCommand =new LatchCommand(m_Pnuematics);

        final zMoveArmRetract RetractArmCommand = new zMoveArmRetract(m_Elevator, m_Rotate);
        JoystickButton Retract=new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorArmReturn);
        Retract.onTrue(RetractArmCommand);

        final SequentialCommandGroup autoMiddleMoveArm =new SequentialCommandGroup( new ClawCommand(m_PnuematicsSubystem, true),
                                                           new zMoveArmExtend(m_Elevator, m_Rotate, Constants.TargetHeight.MIDDLE),
                                                           new ClawCommand(m_PnuematicsSubystem, false),
                                                           new zMoveArmRetract(m_Elevator,m_Rotate));

        JoystickButton moveArmMiddle = new JoystickButton(m_xBoxDriver, ButtonConstants.OperatorAutoMiddle);
        moveArmMiddle.toggleOnTrue(autoMiddleMoveArm);
        final SequentialCommandGroup autoLowMoveArm =new SequentialCommandGroup( new ClawCommand(m_PnuematicsSubystem, true),
                                                           new zMoveArmExtend(m_Elevator, m_Rotate, Constants.TargetHeight.BOTTOM),
                                                           new ClawCommand(m_PnuematicsSubystem, false),
                                                           new zMoveArmRetract(m_Elevator,m_Rotate));

      JoystickButton moveArmLow = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorAutoLow);

      moveArmLow.toggleOnTrue(autoLowMoveArm);
        final SwitchDriveModeCommand switchDriveCommand=new SwitchDriveModeCommand(m_DriveControlMode);  
        
        Trigger driverSpindexerRight=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverSpindexerRight);
        driverSpindexerRight.whileTrue(spindexerRightCommand);

        Trigger driverSpindexerLeft=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverSpindexerLeft);
        driverSpindexerLeft.whileTrue(spindexerLeftCommand);

        Trigger driverIntakeExtend=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverIntakeExtend);
        driverIntakeExtend.whileTrue(ExtendArmsCommand);

        Trigger driverIntakeRetract=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverIntakeRetract);
        driverIntakeRetract.whileTrue(RetractArmsCommand);
        
        Trigger driverMode=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverDriveMode);
        driverMode.toggleOnTrue(switchDriveCommand);
        
        Trigger driverIntakeIn =  new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeIn);
        driverIntakeIn.whileTrue(intakeInCommand);

        Trigger driverIntakeOut =  new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeOut);
        driverIntakeOut.whileTrue(intakeOutCommand);

        Trigger driverGyroReset = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverGyroReset);
        driverGyroReset.whileTrue(gyroResetCommand);

        Trigger driverConveyRetract = new Trigger(() -> Math.abs(m_xBoxDriver.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        driverConveyRetract.toggleOnTrue(intakeConveyandRetract);
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
   
        Trigger operatorClawClose = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorClawClose);
        operatorClawClose.toggleOnTrue(closeClawCommand);

        Trigger operatorClawOpen = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorClawOpen);
        operatorClawOpen.toggleOnTrue(openClawCommand);

        Trigger operatorElevator = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftY())>ButtonConstants.ElevatorDeadBand);
        operatorElevator.whileTrue(elevatorCommand);
        
        Trigger operatorRotate = new Trigger(() -> Math.abs(m_xBoxOperator.getRightX())>ButtonConstants.RotateDeadBand);
        operatorRotate.whileTrue(rotateCommand);

        Trigger operatorTapeAlign = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(2))>ButtonConstants.LeftTriggerDeadBand);
        operatorTapeAlign.toggleOnTrue(tapetargetandMoveCommand);
        
        Trigger operatorTagAlign = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        operatorTagAlign.toggleOnTrue(tagtargetandMoveCommand);

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(Integer selectedMode) {
        Command autoCommand = new AutoDoNothingCommand(); // Default Command is DoNothing
        System.out.println("Autonomouse Selected Mode = " + selectedMode);
        switch (selectedMode) {
          case AutoModes.autoMoveBack:
             autoCommand= new SequentialCommandGroup(new AutoMoveCommand(m_RobotDrive,0,AutoModes.pushDistance),
                                                     new AutoMoveCommand(m_RobotDrive,180, AutoModes.LeaveCommunityDistance));
            break;
          case AutoModes.autoConeLeave:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                          Constants.ChargedUp.GridPosBottomConeAny),
                                                      new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                      new ClawCommand(m_PnuematicsSubystem, false),
                                                      new zPivotArmResetCommand(),
                                                      new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
            break;
          case AutoModes.autoCubeLeave:
            autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                         Constants.ChargedUp.GridPosBottomCubeAny),
                                                     new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                     new ClawCommand(m_PnuematicsSubystem, false),
                                                     new zPivotArmResetCommand(),
                                                     new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
           break;
          case AutoModes.autoConeDock:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomConeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false),
                                                new zPivotArmResetCommand(),
                                                new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
          break;
          case AutoModes.autoCubeDock:
            autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_PnuematicsSubystem, false),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
          break;
          case AutoModes.autoConeEngage:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomConeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_PnuematicsSubystem, false),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                                        new zEngageonChargingCommand());
         
          break;            
          case AutoModes.autoCubeEngage:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_PnuematicsSubystem, false),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                                        new zEngageonChargingCommand());

          break;
          case AutoModes.autoConeScore2:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false),
                                                new zPivotArmResetCommand(),
                                                new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cone),
                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false),
                                                new zPivotArmResetCommand());
         break;
          case AutoModes.autoCubeScore2:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false),
                                                new zPivotArmResetCommand(),
                                                new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_paddle,Constants.ChargedUp.Cube),
                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_PnuematicsSubystem, false),
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

        gryoRoll.set(m_NavX.getAxis(Axis.ROLL));
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
  
    public void LEDAutoMode(){
        m_LEDMode=LEDMode.AUTOMODE;
        LEDUpdate();
        }
    public void TeleopMode(){
        m_LEDMode=LEDMode.TELEOP;  
        LEDUpdate();
        //Set Default Pipeline to AprilTags
        m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
        
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
