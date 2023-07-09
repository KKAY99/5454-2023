// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public class intake{
      public static final int motorPWM=4;
      public static final double speedIn=.5;
      public static final double speedOut=.5;
  }
  public class spindex{
      public static final int motorPWM=5;
      public static final double spinspeed=0.3;
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double speed = 0;

    public static final class InputControllers{
      public static final int kJoystickLeft=0;
      public static final int kJoystickRight=1;
      public static final int kXboxMain=1;
      public static final int kXboxPit=3;
}

    public class ArmSubsystem {
      public static final double homePos=1;
      public static final double shootPos1=1;
      public static final double shootPos2=1;
      public static final double intakePos=1;
    }


    public class zHomeArmCommand {
      public static final double rotateSpeed=0.1;
    }
  }
}
