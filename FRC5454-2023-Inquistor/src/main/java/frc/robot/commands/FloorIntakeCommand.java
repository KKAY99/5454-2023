package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntakeSubsystem;

public class FloorIntakeCommand extends CommandBase{
    private FloorIntakeSubsystem m_intake;
    private double m_speed;

    public FloorIntakeCommand(FloorIntakeSubsystem intake, double speed){
        m_intake = intake;
        m_speed = speed;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
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
