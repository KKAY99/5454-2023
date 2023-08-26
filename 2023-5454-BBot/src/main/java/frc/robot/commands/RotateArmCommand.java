package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;

public class RotateArmCommand extends CommandBase{
    
    private RotateArmSubsystem m_rotateSubsystem;
    private DoubleSupplier m_speed;

    public RotateArmCommand(RotateArmSubsystem rotateSubsystem,DoubleSupplier speed){
        m_rotateSubsystem=rotateSubsystem;
        m_speed=speed;
    }

    @Override
    public void execute(){
        double speed=m_speed.getAsDouble();
        System.out.println("Speed" + speed);
        m_rotateSubsystem.runWithLimits(speed);
    }

    @Override
    public void end(final boolean interrupted){
        m_rotateSubsystem.stop();
    }
}
