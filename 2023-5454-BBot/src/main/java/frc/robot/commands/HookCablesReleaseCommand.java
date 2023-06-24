package frc.robot.commands;


import frc.robot.subsystems.PneumaticsSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HookCablesReleaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PneumaticsSubsystem m_subsystem;
  
  
  /**
    * @param targetSpeed The speed we are setting in execute
   */
  public HookCablesReleaseCommand(PneumaticsSubsystem PneumaticsSystem) {
    m_subsystem=PneumaticsSystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      m_subsystem.setHookCables(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  // Release when button is released 
  if (m_subsystem.getHookCableStatus()){ m_subsystem.setHookCables(false);}
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

