package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FloorIntakeRotateCommand extends CommandBase{
    private ElevatorSubsystem m_elevator;
    private FloorIntakeSubsystem m_intake;
    private double m_speed;

    public FloorIntakeRotateCommand(FloorIntakeSubsystem intake,ElevatorSubsystem elevator,double speed){
        m_intake = intake;
        m_elevator = elevator;
        m_speed = speed;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopRotate();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if(m_elevator.getElevatorPos() < Constants.Lift.posInitLiftRetract){
            m_intake.rotate(m_speed);
        }else{
            m_elevator.SetPosAndMove(Constants.Lift.posInitLiftRetract);
            m_intake.stopRotate();
        }
        return false;
    }
}
