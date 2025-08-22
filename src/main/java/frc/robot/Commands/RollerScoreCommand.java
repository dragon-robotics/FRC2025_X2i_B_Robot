package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

public class RollerScoreCommand extends Command {
    
    private final RollerSubsystem m_roller;
    private double rpm; 
    public RollerScoreCommand(RollerSubsystem m_roller, double rpm)
    {
        this.m_roller = m_roller;
        this.rpm = rpm;
        addRequirements(m_roller);
    }

    @Override 
    public void initialize()
    {
        m_roller.runRoller(rpm);
    }   

    @Override
    public void execute()
    {

    }

    @Override
    public void end(boolean interrupted)
    {
       m_roller.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
