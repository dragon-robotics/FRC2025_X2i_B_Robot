package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends Command
{
    private final ClimberSubsystem m_climber;

    public ClimberUpCommand(ClimberSubsystem climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
    m_climber.runClimber(ClimberConstants.CLIMBER_SPEED_UP);
    }

    @Override
    public void end(boolean interrupted) {
    m_climber.runClimber(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}