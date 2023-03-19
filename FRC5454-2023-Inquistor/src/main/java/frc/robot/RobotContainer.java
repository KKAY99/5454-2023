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
import frc.robot.Constants.TargetHeight;
import frc.robot.Constants.LEDS.Colors;
import frc.robot.classes.DriveControlMode;
import frc.robot.classes.LEDStrip;
import frc.robot.classes.LEDSChargedup;
import frc.robot.classes.Limelight;
import frc.robot.classes.LEDSChargedup.LEDMode;
import frc.robot.commands.*;
import frc.robot.common.drivers.NavX;
import frc.robot.common.drivers.NavX.Axis;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

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
   // private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem(Constants.Spindexer.motorPort);
    private final DrivetrainSubsystem m_RobotDrive = new DrivetrainSubsystem(m_NavX); 
    private final DriveControlMode m_DriveControlMode = new DriveControlMode();
    private final PnuematicsSubystem m_PnuematicsSubystem = new PnuematicsSubystem(Constants.Pneumatics.HubID,
                                                                   Constants.Pneumatics.moduleType,
                                                                   Constants.Pneumatics.clawSolenoid,
                                                                   Constants.Pneumatics.punchSolenoid);
    private final FloorIntakeSubsystem m_FloorIntake = new FloorIntakeSubsystem(Constants.FloorIntake.intakeMotorPort,
                                                            Constants.FloorIntake.rotateMotorPort,
                                                            Constants.FloorIntake.intakeRotateEncoderPort,
                                                            Constants.FloorIntake.rotateLowLimit,
                                                            Constants.FloorIntake.rotateHighLimit);
    private final Limelight m_Limelight = new Limelight(Constants.LimeLightValues.targetHeight, Constants.LimeLightValues.limelightHeight,
                                                 Constants.LimeLightValues.limelightAngle,Constants.LimeLightValues.kVisionXOffset,80);
    private final ElevatorSubsystem m_Elevator = new ElevatorSubsystem(Constants.Elevator.elevatorPort, Constants.Elevator.limitSwitch);
    private final RotateArmSubsystem m_Rotate = new RotateArmSubsystem(Constants.RotateArm.rotateArmPort1,
                                                       Constants.RotateArm.rotateArmPort2,
                                                       Constants.RotateArm.absoluteEncoder,
                                                       Constants.RotateArm.encodervalueHomePos,
                                                       Constants.RotateArm.encoderFrontLimit,
                                                       Constants.RotateArm.encoderBackLimit);


     private final LEDSChargedup m_ledStrip = new LEDSChargedup(Constants.LEDS.UPPERPORT,Constants.LEDS.BOTTOMPORT, Constants.LEDS.UPPERCOUNT,Constants.LEDS.BOTTOMPORT);
     private boolean m_disabled=true;
     private boolean m_homed=false;
     
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
                                            new AutoMoveCommand(m_RobotDrive,0, AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode0,commandAutoMoveBack);

    Command commandScore = new SequentialCommandGroup(new ClawCommand(m_PnuematicsSubystem, true, "auto"),
                                                          new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                                          new ClawCommand(m_PnuematicsSubystem, false,"autoCube"),
                                                          new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem));
    autoChooser.addOption(AutoModes.autoMode1, commandScore);
    
    Command commandAutoScoreLeave = new SequentialCommandGroup(new ClawCommand(m_PnuematicsSubystem, true, "auto"),
    new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                                              new ClawCommand(m_PnuematicsSubystem, false, "auto"),
                                                              new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem),
                                                              new AutoMoveCommand(m_RobotDrive,0,AutoModes.LeaveCommunityDistance));
    autoChooser.addOption(AutoModes.autoMode2,commandAutoScoreLeave);
 
    Command commandAutoScoreDock = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomConeAny,m_xBoxDriver),
                                               new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                               new ClawCommand(m_PnuematicsSubystem, false,"autoCone"),
                                               new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem),
                                               new AutoMoveCommand(m_RobotDrive,0,AutoModes.DistanceToDock));
    autoChooser.addOption(AutoModes.autoMode3,commandAutoScoreDock);
 
 
    Command commandAutoScoreEngage  = new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                       Constants.ChargedUp.GridPosBottomConeAny,m_xBoxDriver),
                                                       new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                                       new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                                       new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem),
                                                       new AutoMoveCommand(m_RobotDrive,0,AutoModes.DistanceToCharging),
                                                       new zEngageonChargingCommand());
   autoChooser.addOption(AutoModes.autoMode4,commandAutoScoreEngage);

 
   Command commandAutoScore2= new SequentialCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny,m_xBoxDriver),
                                               new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                               new ClawCommand(m_PnuematicsSubystem, true,"auto"),
                                               new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem),
 //                                      new zAutoDetectandGetCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.Cone),
                                        new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                               Constants.ChargedUp.GridPosBottomCubeAny,m_xBoxDriver),
                                               new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                       new ClawCommand(m_PnuematicsSubystem, false,"auto"),
                                       new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem));
   autoChooser.addOption(AutoModes.autoMode5, commandAutoScore2);
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
        final GyroResetCommand gyroResetCommand = new GyroResetCommand(m_RobotDrive,m_Limelight);

        final zAutoTargetandMoveCommand pipelineAprilTagCommand = new zAutoTargetandMoveCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosMiddleCenter,m_xBoxDriver);
        final zAutoTargetandMoveCommand pipelineTapeLowCommand = new zAutoTargetandMoveCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosMiddleConeAny,m_xBoxDriver);
        final zAutoTargetandMoveCommand pipelineTapeHighCommand = new zAutoTargetandMoveCommand(m_Limelight,m_RobotDrive,Constants.ChargedUp.GridPosUpperConeAny,m_xBoxDriver);

        final RotateCommand rotateCommand = new RotateCommand(m_Rotate,() -> (m_xBoxOperator.getLeftX()),Constants.RotateArm.manualLimitSpeed);
        final ElevatorCommand elevatorCommand = new ElevatorCommand(m_Elevator,m_Rotate,() -> (m_xBoxOperator.getLeftY()), Constants.Elevator.elevatorLimitSpeed);
        final FloorIntakeCommand floorIntakeInCommand = new FloorIntakeCommand(m_FloorIntake,m_Elevator,Constants.FloorIntake.intakeSpeed,Constants.FloorIntake.intakeRotateSpeed,
                                                                            Constants.Lift.posLiftOutfIntake, FloorIntakeCommand.STATE.AUTOINTAKE,0);
        final FloorIntakeCommand floorIntakeOutCommand = new FloorIntakeCommand(m_FloorIntake,m_Elevator,Constants.FloorIntake.intakeSpeed,Constants.FloorIntake.intakeRotateSpeed,
                                                                                Constants.Lift.posLiftOutfIntake,FloorIntakeCommand.STATE.AUTOEXHAUST,0);
        final FloorIntakeRotateCommand floorRotateUpCommand = new FloorIntakeRotateCommand(m_FloorIntake,m_Elevator,Constants.FloorIntake.intakeRotateSpeed,Constants.Lift.posLiftOutfIntake);
        final FloorIntakeRotateCommand floorRotateDownCommand = new FloorIntakeRotateCommand(m_FloorIntake,m_Elevator,-Constants.FloorIntake.intakeRotateSpeed,Constants.Lift.posLiftOutfIntake);
        final FloorIntakeCommand floorIntakeInOCommand = new FloorIntakeCommand(m_FloorIntake,m_Elevator,Constants.FloorIntake.intakeSpeed,Constants.FloorIntake.intakeRotateSpeed,
                                                              Constants.Lift.posLiftOutfIntake,FloorIntakeCommand.STATE.AUTOINTAKE,0);
        final FloorIntakeCommand floorIntakeOutOCommand = new FloorIntakeCommand(m_FloorIntake,m_Elevator,Constants.FloorIntake.intakeSpeed,
                                                              Constants.Lift.posLiftOutfIntake,Constants.FloorIntake.intakeRotateSpeed,FloorIntakeCommand.STATE.AUTOEXHAUST,0);
        final FloorIntakeRotateCommand floorRotateUpOCommand = new FloorIntakeRotateCommand(m_FloorIntake,m_Elevator,Constants.FloorIntake.intakeRotateSpeed,Constants.Lift.posLiftOutfIntake);
        final FloorIntakeRotateCommand floorRotateDownOCommand = new FloorIntakeRotateCommand(m_FloorIntake,m_Elevator,-Constants.FloorIntake.intakeRotateSpeed,Constants.Lift.posLiftOutfIntake);
        
        final ClawSwapCommand swapClawCommand = new ClawSwapCommand(m_PnuematicsSubystem);
        final SolenoidPunchCommand punchSolenoidCommand = new SolenoidPunchCommand(m_PnuematicsSubystem);
        // Auto commands
        final zBalanceRobotCommand balanceRobotCommand = new zBalanceRobotCommand(m_NavX,m_RobotDrive);
        final zMoveArmRetractABS retractCommand = new zMoveArmRetractABS(m_Elevator, m_Rotate,m_PnuematicsSubystem);
        final SequentialCommandGroup retractElevatorCommand = new SequentialCommandGroup(new ColorSwapCommand(m_ledStrip, LEDMode.RETRACTING),
                                                                                retractCommand,new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));

        Trigger wristArm = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorWrist);
        wristArm.toggleOnTrue(punchSolenoidCommand);
        Trigger customRetract = new JoystickButton(m_CustomController,ButtonConstants.CustomCtlRetract);
        customRetract.toggleOnTrue(new zMoveArmRetractABS(m_Elevator, m_Rotate,m_PnuematicsSubystem));

        // Custom Controller Parallel Scoring Commands 
        final ParallelCommandGroup zAutoTargetTL= new ParallelCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperLeft,m_xBoxDriver),
                                                                                new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, true,true));
        final SequentialCommandGroup zAutoTargetTLMaster= new SequentialCommandGroup(new zMoveElevatorPIDCommand(m_Elevator,Constants.Lift.topTape),
                                                                                     new zPipelaneSwapCommand(m_Limelight,Constants.ChargedUp.GridPosUpperLeft),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                                                     zAutoTargetTL,new ClawCommand(m_PnuematicsSubystem, m_Limelight,false),
                                                                                     new zMoveArmRetractABS(m_Elevator, m_Rotate, m_PnuematicsSubystem),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));                                                                              
        Trigger targetTopLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetTopLeft);
        targetTopLeft.toggleOnTrue(zAutoTargetTLMaster);

        final ParallelCommandGroup zAutoTargetML= new ParallelCommandGroup(new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleLeft,m_xBoxDriver),
                                                                                new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.MIDDLE, true,true));
        final SequentialCommandGroup zAutoTargetMLMaster= new SequentialCommandGroup(new zMoveElevatorPIDCommand(m_Elevator,Constants.Lift.middleTape),
                                                                                     new zPipelaneSwapCommand(m_Limelight,Constants.ChargedUp.GridPosMiddleLeft),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                                                     zAutoTargetML,new ClawCommand(m_PnuematicsSubystem, m_Limelight, false),
                                                                                     new zMoveArmRetractABS(m_Elevator, m_Rotate, m_PnuematicsSubystem),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
         Trigger targetMiddleLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleLeft);
        targetMiddleLeft.toggleOnTrue(zAutoTargetMLMaster);

        final ParallelCommandGroup zAutoTargetBL= new ParallelCommandGroup(new zMoveElevatorPIDCommand(m_Elevator,Constants.Lift.middleTape),
                                                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosBottomLeft,m_xBoxDriver),
                                                                                new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.BOTTOM, true,false));
        final SequentialCommandGroup zAutoTargetBLMaster= new SequentialCommandGroup(new zPipelaneSwapCommand(m_Limelight,Constants.ChargedUp.GridPosBottomLeft),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                                                     zAutoTargetBL,new ClawCommand(m_PnuematicsSubystem, m_Limelight, false),
                                                                                     new zMoveArmRetractABS(m_Elevator, m_Rotate, m_PnuematicsSubystem),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
        Trigger targetBottomLeft= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomLeft);
        targetBottomLeft.toggleOnTrue(zAutoTargetBLMaster);

        final ParallelCommandGroup zAutoTargetTC= new ParallelCommandGroup(new zMoveElevatorPIDCommand(m_Elevator,Constants.Lift.apriltag),
                                                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosUpperCenter,m_xBoxDriver),
                                                                                new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, true,true));
        final SequentialCommandGroup zAutoTargetTCMaster= new SequentialCommandGroup(new zPipelaneSwapCommand(m_Limelight,Constants.ChargedUp.GridPosUpperCenter),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                                                     zAutoTargetTC,new ClawCommand(m_PnuematicsSubystem, m_Limelight,false),
                                                                                     new zMoveArmRetractABS(m_Elevator, m_Rotate, m_PnuematicsSubystem),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
        Trigger targetTopCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetTopCenter);
        targetTopCenter.toggleOnTrue(zAutoTargetTCMaster);

        final ParallelCommandGroup zAutoTargetMC= new ParallelCommandGroup(new zMoveElevatorPIDCommand(m_Elevator,Constants.Lift.apriltag),
                                                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive ,
                                                                                Constants.ChargedUp.GridPosMiddleCenter,m_xBoxDriver),
                                                                                new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.MIDDLE, true,true));
        final SequentialCommandGroup zAutoTargetMCMaster= new SequentialCommandGroup(new zPipelaneSwapCommand(m_Limelight,Constants.ChargedUp.GridPosMiddleCenter),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                                                     zAutoTargetMC,new ClawCommand(m_PnuematicsSubystem, m_Limelight, false),
                                                                                     new zMoveArmRetractABS(m_Elevator, m_Rotate, m_PnuematicsSubystem),
                                                                                     new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
        Trigger targetMiddleCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleCenter);
        targetMiddleCenter.toggleOnTrue(zAutoTargetMCMaster);

        final ParallelCommandGroup zAutoTargetBC= new ParallelCommandGroup(new zMoveElevatorPIDCommand(m_Elevator,Constants.Lift.apriltag),
                                                                                new zAutoTargetandMoveCommand(m_Limelight, m_RobotDrive,
                                                                                Constants.ChargedUp.GridPosBottomCenter,m_xBoxDriver),
                                                                                new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.BOTTOM, true,false));
        final SequentialCommandGroup zAutoTargetBCMaster= new SequentialCommandGroup(new zPipelaneSwapCommand(m_Limelight,Constants.ChargedUp.GridPosBottomCenter),
                                                                                new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                                                zAutoTargetBC,new ClawCommand(m_PnuematicsSubystem, m_Limelight, false),
                                                                                new zMoveArmRetractABS(m_Elevator, m_Rotate, m_PnuematicsSubystem),
                                                                                new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
        Trigger targetBottomCenter= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomCenter);
        targetBottomCenter.toggleOnTrue(zAutoTargetBCMaster);

        final ParallelCommandGroup conePipelineTopCone = new ParallelCommandGroup(new zPipelaneSwapCommand(m_Limelight, Constants.ChargedUp.GridPosUpperConeAny));
        Trigger topConeLED= new JoystickButton(m_CustomController,ButtonConstants.TargetTopRight);
        topConeLED.toggleOnTrue(conePipelineTopCone);

        final ParallelCommandGroup conePipelineMiddleCone = new ParallelCommandGroup(new zPipelaneSwapCommand(m_Limelight, Constants.ChargedUp.GridPosUpperConeAny));
        Trigger middleConeLED= new JoystickButton(m_CustomController,ButtonConstants.TargetMiddleRight);
        middleConeLED.toggleOnTrue(conePipelineMiddleCone);

        final ParallelCommandGroup cubePipelineLED = new ParallelCommandGroup(new zPipelaneSwapCommand(m_Limelight, Constants.ChargedUp.GridPosUpperConeAny));
        Trigger cubeLED= new JoystickButton(m_CustomController,ButtonConstants.TargetBottomRight);
        cubeLED.toggleOnTrue(cubePipelineLED);

        //final LatchCommand latchCommand =new LatchCommand(m_Pnuematics);
        Trigger swapClaw = new JoystickButton(m_xBoxOperator,ButtonConstants.OperatorClawSwap);
        swapClaw.toggleOnTrue(swapClawCommand);

        final SequentialCommandGroup autoLowMoveArm =new SequentialCommandGroup(//new ClawCommand(m_PnuematicsSubystem, true),
        //                                                   new PaddleMoveToCommand(m_paddle,Constants.Paddle.encoderMovePosLowShot,Constants.Paddle.autoMoveTolerance,Constants.Paddle.autoMoveOutSpeed),
                                                           new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                           new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.BOTTOM, false,false),
                                                           new ClawCommand(m_PnuematicsSubystem, false,"zAuto"),
                                                           new zMoveArmRetractABS(m_Elevator,m_Rotate,m_PnuematicsSubystem),
                                                           new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));

        JoystickButton moveArmLow = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorAutoLow);
        moveArmLow.toggleOnTrue(autoLowMoveArm);

        final SequentialCommandGroup autoMiddleMoveArm =new SequentialCommandGroup(//new ClawCommand(m_PnuematicsSubystem, true),
         //                                                  new PaddleMoveToCommand(m_paddle,Constants.Paddle.encoderHumanPlayerPos,Constants.Paddle.autoMoveTolerance,Constants.Paddle.autoMoveOutSpeed),
                                                            new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                            new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.MIDDLE, false,true),
                                                            new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
                                                           // ClawCommand(m_PnuematicsSubystem, false),
                                                           //new zMoveArmRetractABS(m_Elevator,m_Rotate)

        JoystickButton moveArmMiddle = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorAutoMiddle);
        moveArmMiddle.toggleOnTrue(autoMiddleMoveArm);

        final SequentialCommandGroup autoHighMoveArm =new SequentialCommandGroup(//new ClawCommand(m_PnuematicsSubystem, true),
         //                                                  new PaddleMoveToCommand(m_paddle,Constants.Paddle.encoderHumanPlayerPos,Constants.Paddle.autoMoveTolerance,Constants.Paddle.autoMoveOutSpeed),
                                                             new ColorSwapCommand(m_ledStrip, LEDMode.AUTOSCORING),
                                                             new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.TOP, false,true),
                                                             new ColorSwapCommand(m_ledStrip, LEDMode.TELEOP));
                                                           //new ClawCommand(m_PnuematicsSubystem, false,"zAuto"),
                                                           //new zMoveArmRetractABS(m_Elevator,m_Rotate)
        
        Trigger moveArmHigh = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        moveArmHigh.toggleOnTrue(autoHighMoveArm);
                                                           
        final SequentialCommandGroup playerStationGrab = new SequentialCommandGroup(new zMoveArmExtendABS(m_Elevator, m_Rotate, m_PnuematicsSubystem,m_Limelight,Constants.TargetHeight.PLAYERSTATION,false,false),
                                                                                    new ClawCommand(m_PnuematicsSubystem, false,"zAuto"));

        Trigger playerGrab = new Trigger(() -> Math.abs(m_xBoxOperator.getRawAxis(2))>Constants.ButtonConstants.LeftTriggerDeadBand);
        playerGrab.toggleOnTrue(playerStationGrab);

        final SwitchDriveModeCommand switchDriveCommand=new SwitchDriveModeCommand(m_DriveControlMode);  
        
        Trigger driverMode=new JoystickButton(m_xBoxDriver,ButtonConstants.DriverDriveMode);
        driverMode.toggleOnTrue(switchDriveCommand);
        
        Trigger driverSolenoidPunch =  new JoystickButton(m_xBoxDriver, ButtonConstants.DriverSolenoidPunch);
        driverSolenoidPunch.toggleOnTrue(punchSolenoidCommand);

        Trigger driverGyroReset = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverGyroReset);
        driverGyroReset.whileTrue(gyroResetCommand);

        Trigger driverLowTape = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverPipelineLowTape);
        driverLowTape.toggleOnTrue(pipelineTapeLowCommand);

        Trigger driverHighTape = new JoystickButton(m_xBoxDriver, ButtonConstants.DriverPipelineHighTape);
        driverHighTape.toggleOnTrue(pipelineTapeHighCommand);
        
        Trigger driverPipelineAprilTag = new Trigger(() -> Math.abs(m_xBoxDriver.getRawAxis(3))>ButtonConstants.RightTriggerDeadBand);
        driverPipelineAprilTag.toggleOnTrue(pipelineAprilTagCommand);

        Trigger driverAutoBalance = new JoystickButton(m_xBoxDriver,ButtonConstants.DriverAutoBalance);
        driverAutoBalance.whileTrue(balanceRobotCommand);

        Trigger operatorElevator = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftY())>ButtonConstants.ElevatorDeadBand);
        operatorElevator.whileTrue(elevatorCommand);
        
        Trigger operatorRotate = new Trigger(() -> Math.abs(m_xBoxOperator.getLeftX())>ButtonConstants.RotateDeadBand);
        operatorRotate.whileTrue(rotateCommand);

        //Trigger operatorIntake = new Trigger(() -> Math.abs(m_xBoxOperator.getRightX())<ButtonConstants.IntakeDeadBand);
       // operatorIntake.whileTrue(floorIntakeCommand);

        
//        Trigger operatorIntakeRotate = new Trigger(() -> Math.abs(m_xBoxOperator.getRightY())>ButtonConstants.IntakeRotateDeadBand);
  //      operatorIntakeRotate.whileTrue(floorIntakeRotateCommand);
        Trigger operatorIntakeIn = new POVButton(m_xBoxOperator,Constants.ButtonConstants.OperatorPOVIntakeIn);
        operatorIntakeIn.whileTrue(floorIntakeInOCommand);
        Trigger operatorIntakeOut = new POVButton(m_xBoxOperator,Constants.ButtonConstants.OperatorPOVIntakeOut);
        operatorIntakeOut.whileTrue(floorIntakeOutOCommand);
                
        Trigger operatorIntakeRotateUp = new POVButton(m_xBoxOperator, Constants.ButtonConstants.OperatorPOVRotateUp);
         operatorIntakeRotateUp.whileTrue(floorRotateUpOCommand);

         Trigger operatorIntakeRotateDown = new POVButton(m_xBoxOperator, Constants.ButtonConstants.OperatorPOVRotatedown);
         operatorIntakeRotateDown.whileTrue(floorRotateDownOCommand);

         Trigger driverIntakeIn = new POVButton(m_xBoxDriver,Constants.ButtonConstants.OperatorPOVIntakeIn);
         driverIntakeIn.whileTrue(floorIntakeInCommand);
         Trigger driverIntakeOut = new POVButton(m_xBoxDriver,Constants.ButtonConstants.OperatorPOVIntakeOut);
         driverIntakeOut.whileTrue(floorIntakeOutCommand);
                 
         Trigger driverIntakeRotateUp = new POVButton(m_xBoxDriver, Constants.ButtonConstants.OperatorPOVRotateUp);
          driverIntakeRotateUp.whileTrue(floorRotateUpCommand);
 
          Trigger driverIntakeRotateDown = new POVButton(m_xBoxDriver, Constants.ButtonConstants.OperatorPOVRotatedown);
          driverIntakeRotateDown.whileTrue(floorRotateDownCommand);
 

        Trigger operatorAutoRetractElevator = new JoystickButton(m_xBoxOperator, ButtonConstants.OperatorRetractElevator);
        operatorAutoRetractElevator.toggleOnTrue(retractElevatorCommand);

    }
       
    public void refreshSmartDashboard()
    {  
        frontLeftAngle.set(m_RobotDrive.getFrontLeftAngle());
        frontRightAngle.set(m_RobotDrive.getFrontRightAngle());
        backLeftAngle.set(m_RobotDrive.getBackLeftAngle());
        backRightAngle.set(m_RobotDrive.getbackRightAngle());
        elevatorEncoder.set(m_Elevator.getElevatorPos());
        rotateEncoder.set(m_Rotate.getRelativeRotatePos());
        elevatorLimitSwitch.set(m_Elevator.hasHitPhysicalLimitSwitch());
        rotateBackSoftLimit.set(m_Rotate.hasHitBackSoftLimit());
        rotateForwardSoftLimit.set(m_Rotate.hasHitForwardSoftLimit());
        elevatorMaxLimit.set(m_Elevator.hasHitMaxLimit());
        gryoRoll.set(m_NavX.getAxis(Axis.ROLL));
        compressorPressure.set(m_PnuematicsSubystem.getPressure());

        m_ledStrip.m_pipeline = m_Limelight.getPipeline();
        m_ledStrip.m_canSeeTarget = m_Limelight.isTargetAvailible();

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


        if(m_Limelight.getPipeline()==Constants.VisionPipelines.AprilTag){
                m_ledStrip.setPipelineLED();
        }else{
                if(m_Limelight.getPipeline()==Constants.VisionPipelines.BottomTape||m_Limelight.getPipeline()==Constants.VisionPipelines.TopTape){
                        m_ledStrip.setPipelineLED();
                }
        }

        //override disabled led mode
        if(m_disabled){
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.DISABLED);
        }
        m_ledStrip.updateLED();
        
}
    
     

     public void disabledPerioidicUpdates(){
        m_ledStrip.updateLED();
       

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
   
    
  
        public void AutoMode(){
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.AUTOMODE);
                homeRobot();
                //Set Default Pipeline to AprilTags
                m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
                
        }  
        public void TeleopMode(){
                m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.TELEOP);
                homeRobot();
                //Set Default Pipeline to AprilTags
                m_Limelight.setPipeline(Constants.VisionPipelines.AprilTag);
                
        }
   
    private void homeRobot(){
        
        if(m_homed==false){
                //zHomeRotateCommand resetRotate = new zHomeRotateCommand(m_Rotate,Constants.RotateArm.encodervalueHomePos, Constants.Rotate.homeSpeedForward,Constants.Rotate.homeTimeFailsafe) ; 
                zHomeElevatorCommand resetElevator = new zHomeElevatorCommand(m_Elevator,Constants.Elevator.elevatorLimitSpeed,Constants.Elevator.homeTimeOut) ; 
                ClawCommand homeCloseClawCommand = new ClawCommand(m_PnuematicsSubystem, true,"homeRobot"); 
                ParallelCommandGroup resetRobot = new ParallelCommandGroup(resetElevator,homeCloseClawCommand); 
               // FloorIntakeHomeCommand intakeHome = new FloorIntakeHomeCommand(m_FloorIntake,Constants.FloorIntake.intakeRotateSpeed,Constants.FloorIntake.homeTimeOut);                                                              
                CommandScheduler.getInstance().schedule(resetRobot);
               // CommandScheduler.getInstance().schedule(intakeHome);

                m_homed=true;
        } 
    }
    
    public void DisableMode(){
            m_disabled=true;
            m_ledStrip.setRobotMode(LEDSChargedup.LEDMode.DISABLED);
       
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
