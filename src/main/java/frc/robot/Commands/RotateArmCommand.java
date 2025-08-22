
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RotateArmCommand extends Command{
    
private final ArmSubsystem m_arm;
public RotateArmCommand(ArmSubsystem m_arm)
{
    this.m_arm = m_arm;
    addRequirements(m_arm);
}

@Override
public void initialize() {
    m_arm.pivotArm();

}

@Override
public void execute()
{
    
}


@Override
public void end(boolean interrupted)
{
}
}