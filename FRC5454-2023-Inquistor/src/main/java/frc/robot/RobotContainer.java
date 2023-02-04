// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

 
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import frc.robot.commands.GyroResetCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SwitchDriveModeCommand;
import frc.robot.commands.zAutoDetectandGetCommand;
import frc.robot.commands.zAutoTargetToColumnCommand;
import frc.robot.commands.zAutoTargetandMoveCommand;
import frc.robot.commands.zBalanceRobotCommand;
import frc.robot.commands.zEngageonChargingCommand;
import frc.robot.commands.zPivotArmResetCommand;
import frc.robot.commands.zPivotandExtendCommand;
import frc.robot.commands.zEngageonChargingCommand;
import frc.robot.common.drivers.NavX;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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
  
    private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
    private final ClawSubsystem m_Claw = new ClawSubsystem(Constants.Claw.clawPort); 
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight, Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset,80);
    
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
     private final IntakeSubsystem m_intake = new IntakeSubsystem(Constants.Intake.intakePort);
   
    private final PowerDistribution m_robotPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);


    private static ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
    private static ShuffleboardTab SwerveEncoders = Shuffleboard.getTab("SwerveEncoders");
    private static ShuffleboardTab AutoTab = Shuffleboard.getTab("Auto");
    private static ShuffleboardTab TargetingTab = Shuffleboard.getTab("Targeting");
   
    
    static GenericEntry networkTableEntryVisionDistance = TargetingTab.add("Vision Distance", 0)
            .withWidget(BuiltInWidgets.kNumberBar).withSize(2, 2).getEntry();

    
    static GenericEntry networkTableEntryFrontLeftSpeed = SwerveTab.add("FL Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(1, 0).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryFrontRightSpeed = SwerveTab.add("FR Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(14, 0).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryBackLeftSpeed = SwerveTab.add("BL Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(1, 5).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryBackRightSpeed = SwerveTab.add("BR Speed", 0)
            .withWidget(BuiltInWidgets.kVoltageView)
            .withProperties(Map.of("Min", 0, "Max", 1, "Center", 0, "Orientation", "VERTICAL"))
            .withPosition(14, 5).withSize(2, 5).getEntry();

    static GenericEntry networkTableEntryFrontLeftEncoderActual = SwerveEncoders.add("FL Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(0, 0).withSize(2, 1).getEntry();
 
    static GenericEntry networkTableEntryFrontRightEncoderActual = SwerveEncoders.add("FR Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 0).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackLeftEncoderActual = SwerveEncoders.add("BL Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(0, 1).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackRightEncoderActual = SwerveEncoders.add("BR Encoder Actual", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 1).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryFrontLeftEncoderTarget = SwerveEncoders.add("FL Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryFrontRightEncoderTarget = SwerveEncoders.add("FR Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(7, 0).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackLeftEncoderTarget = SwerveEncoders.add("BL Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(5, 1).withSize(2, 1).getEntry();

    static GenericEntry networkTableEntryBackRightEncoderTarget = SwerveEncoders.add("BR Encoder Target", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(7, 1).withSize(2, 1).getEntry();

    static GenericEntry frontLeftAngle = SwerveEncoders.add("FL Angle", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 3).withSize(2, 1).getEntry();
    static GenericEntry frontRightAngle = SwerveEncoders.add("FR Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(2, 3).withSize(2, 1).getEntry();
    static GenericEntry backLeftAngle = SwerveEncoders.add("BL Angle", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4).withSize(2, 1).getEntry();
    static GenericEntry backRightAngle = SwerveEncoders.add("BR Angle", 0).withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 4).withSize(2, 1).getEntry();

    static GenericEntry frontLeft360Angle = SwerveEncoders.add("FL 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(4, 3).withSize(2, 1).getEntry();
    static GenericEntry frontRight360Angle = SwerveEncoders.add("FR 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(6, 3).withSize(2, 1).getEntry();
    static GenericEntry backLeft360Angle = SwerveEncoders.add("BL 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(4, 4).withSize(2, 1).getEntry();
    static GenericEntry backRight360Angle = SwerveEncoders.add("BR 360 Angle", 0)
            .withWidget(BuiltInWidgets.kTextView).withPosition(6, 4).withSize(2, 1).getEntry();

    static GenericEntry ShuffleboardLog = SwerveEncoders.add("ShuffleboardLog", "")
            .withWidget(BuiltInWidgets.kTextView).withSize(4, 2).withPosition(0, 6).getEntry();

    static GenericEntry shuffleboardGyroFused = SwerveTab.add("Gyro - Fused Heading", 0)
            .withWidget(BuiltInWidgets.kTextView).getEntry();

    static GenericEntry shuffleboardRobotMoving=SwerveTab.add("Robot Moving","")
                               .getEntry();
 
    static String ShuffleboardLogString;
     
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
                                            new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                            new zPivotArmResetCommand(),
                                            new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode3,commandAutoCubeLeave);
 
 Command commandAutoConeDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomConeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                       new zPivotArmResetCommand(),
                                       new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
 autoChooser.addOption(AutoModes.autoMode4,commandAutoConeDock);
 
 Command commandAutoCubeDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomCubeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
 autoChooser.addOption(AutoModes.autoMode5, commandAutoCubeDock);
 
 Command commandAutoConeEngage  = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomConeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                               new zEngageonChargingCommand());

autoChooser.addOption(AutoModes.autoMode6,commandAutoConeEngage);
Command commandAutoCubeEngage = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomCubeAny),
                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                               new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                               new zPivotArmResetCommand(),
                                               new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                               new zEngageonChargingCommand());

autoChooser.addOption(AutoModes.autoMode7,commandAutoCubeEngage);
 
Command commandAutoConeScore2= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                       new zPivotArmResetCommand(),
                                       new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_intake,Constants.ChargedUp.Cone),
                                       new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                       new zPivotArmResetCommand());
 autoChooser.addOption(AutoModes.autoMode8, commandAutoConeScore2);
 Command commandAutoCubeScore2 = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                       new zPivotArmResetCommand(),
                                       new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_intake,Constants.ChargedUp.Cube),
                                       new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny),
                                       new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                       new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
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
        final IntakeCommand intakeInCommand = new IntakeCommand(m_intake, Constants.Intake.intakeInSpeed);
        final IntakeCommand intakeOutCommand = new IntakeCommand(m_intake, Constants.Intake.intakeOutSpeed);
        final GyroResetCommand gyroResetCommand = new GyroResetCommand(m_RobotDrive,m_Limelight);


        final zBalanceRobotCommand balanceRobotCommand = new zBalanceRobotCommand(m_NavX,m_RobotDrive);
        final zAutoTargetToColumnCommand targetColumnCommand = new zAutoTargetToColumnCommand(m_Limelight,m_RobotDrive,0);
        Trigger balanceRobot=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverBalance);
        balanceRobot.whileTrue(balanceRobotCommand);

        // *** 9 Box targeting
        final SequentialCommandGroup zAutoTargetTL= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                              new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetTopLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetTopLeft);
        targetTopLeft.toggleOnTrue(zAutoTargetTL);

        final SequentialCommandGroup zAutoTargetML= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                              new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetMiddleLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleLeft);
        targetMiddleLeft.toggleOnTrue(zAutoTargetML);

        final SequentialCommandGroup zAutoTargetBL= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosBottomLeft),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                              new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetMiddleBottom= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomLeft);
        targetMiddleBottom.toggleOnTrue(zAutoTargetBL);

        final SequentialCommandGroup zAutoTargetTC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                              new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetTopCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetTopCenter);
        targetTopLeft.toggleOnTrue(zAutoTargetTC);

        final SequentialCommandGroup zAutoTargetMC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                              new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetMiddleCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleCenter);
        targetMiddleCenter.toggleOnTrue(zAutoTargetMC);

        final SequentialCommandGroup zAutoTargetBC= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosBottomCenter),
                                                                              new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                              new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetBottomCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomCenter);
        targetBottomCenter.toggleOnTrue(zAutoTargetBC);

        final SequentialCommandGroup zAutoTargetTR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosUpperRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                                               new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));

        Trigger targetTopRight =new JoystickButton(m_CustomController,ButtonConstants.TargetTopRight);
        targetTopRight.toggleOnTrue(zAutoTargetTR);

        final SequentialCommandGroup zAutoTargetMR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.MIDDLE),
                                                                               new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetMiddleRight= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleRight);
        targetMiddleRight.toggleOnTrue(zAutoTargetMR);

        final SequentialCommandGroup zAutoTargetBR= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosBottomRight),
                                                                               new zPivotandExtendCommand(Constants.TargetHeight.BOTTOM),
                                                                               new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed));
        Trigger targetBottomRight= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomRight);
        targetBottomRight.toggleOnTrue(zAutoTargetBR);

        //final LatchCommand latchCommand =new LatchCommand(m_Pnuematics);
        final SwitchDriveModeCommand switchDriveCommand=new SwitchDriveModeCommand(m_DriveControlMode);       
        Trigger driverMode=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverDriveMode);
        driverMode.toggleOnTrue(switchDriveCommand);
        
        Trigger driverIntakeIn =  new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeIn);
        driverIntakeIn.whileTrue(intakeInCommand);

        Trigger driverIntakeOut =  new JoystickButton(m_xBoxDriver, ButtonConstants.DriverIntakeOut);
        driverIntakeOut.whileTrue(intakeOutCommand);

     
        Trigger driverGyroReset = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverGyroReset);
        driverGyroReset.whileTrue(gyroResetCommand);
/*            
        Trigger operatorIntakeIn =  new JoystickButton(m_xBoxDriver, ButtonConstants.OperatorIntakeIn);
        operatorIntakeIn.whileTrue(intakeInCommand);

        Trigger operatorIntakeOut =  new JoystickButton(m_xBoxDriver, ButtonConstants.OperatorIntakeOut);
        operatorIntakeOut.whileTrue(intakeInCommand);

        Trigger operatorGyroReset = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorGyroReset);
        operatorGyroReset.whileTrue(gyroResetCommand);
 */       
        
 
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
                                                      new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                      new zPivotArmResetCommand(),
                                                      new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
            break;
          case AutoModes.autoCubeLeave:
            autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                         Constants.ChargedUp.GridPosBottomCubeAny),
                                                     new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                     new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                     new zPivotArmResetCommand(),
                                                     new AutoMoveCommand(m_RobotDrive,180,AutoModes.LeaveCommunityDistance));
           break;
          case AutoModes.autoConeDock:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomConeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                new zPivotArmResetCommand(),
                                                new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
          break;
          case AutoModes.autoCubeDock:
            autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToDock));
          break;
          case AutoModes.autoConeEngage:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomConeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                                        new zEngageonChargingCommand());
         
          break;            
          case AutoModes.autoCubeEngage:
          autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                Constants.ChargedUp.GridPosBottomCubeAny),
                                                        new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                        new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                        new zPivotArmResetCommand(),
                                                        new AutoMoveCommand(m_RobotDrive,180,AutoModes.DistanceToCharging),
                                                        new zEngageonChargingCommand());

          break;
          case AutoModes.autoConeScore2:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                new zPivotArmResetCommand(),
                                                new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_intake,Constants.ChargedUp.Cone),
                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                new zPivotArmResetCommand());
         break;
          case AutoModes.autoCubeScore2:
             autoCommand = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                new zPivotArmResetCommand(),
                                                new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,m_intake,Constants.ChargedUp.Cube),
                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                        Constants.ChargedUp.GridPosBottomCubeAny),
                                                new zPivotandExtendCommand(Constants.TargetHeight.TOP),
                                                new ClawCommand(m_Claw,Constants.Claw.ReleaseSpeed),
                                                new zPivotArmResetCommand());
                                                
          default:
            autoCommand = new AutoDoNothingCommand();
        }
        return autoCommand;
        // return m_autoCommand;
    }   
    public void refreshSmartDashboard()
    {  
        frontLeftAngle.setDouble(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.setDouble(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.setDouble(m_RobotDrive.getBackLeftAngle());
        backRightAngle.setDouble(m_RobotDrive.getbackRightAngle());
        m_Limelight.update();
        //override disabled led mode
        if(m_disabled){
                m_LEDMode=LEDMode.DISBLED;
        }
        LEDUpdate();
        updateRobotMoving();
      
}
    
     private void updateRobotMoving(){
                if (m_RobotDrive.IsRobotMoving()){
                        shuffleboardRobotMoving.setString("True");
                }
                else{
                        shuffleboardRobotMoving.setString("False");     
                }
        }

     public void disabledPerioidicUpdates(){
        LEDUpdate();
        updateRobotMoving();

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
