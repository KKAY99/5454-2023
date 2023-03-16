package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class FloorIntakeCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private Double m_speed;

    public FloorIntakeCommand(FloorIntakeSubsystem intake, Double speed){
        m_intake = intake;
        m_speed = speed;
        //Allow both flor commands to use intake since thy are using different motors
        //addRequirements(m_intake);
    }

    @Override
    public void execute(){
        System.out.println("FlrIntake " + m_speed);
            m_intake.runIntake(m_speed);

    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
