package frc.robot.Commands.AlgaeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

public class RollerIntakeCommand extends Command {
    private RollerSubsystem m_roller;
    private double rpm; 
    public RollerIntakeCommand(RollerSubsystem m_roller, double rpm) {
        this.m_roller = m_roller;
        this.rpm = rpm; 
        addRequirements(m_roller);
    }

    @Override
    public void initialize() {
        m_roller.runRoller(-rpm);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_roller.stopRoller();
    }

}