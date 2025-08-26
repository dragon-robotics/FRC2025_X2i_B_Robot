package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class ClimberSubsystem extends SubsystemBase 
{
    private final TalonFX climbMotor; 
    private TalonFXConfiguration talonConfig;
    
    private final DutyCycleOut m_cycleOut;

    public ClimberSubsystem()
    {
        climbMotor = new TalonFX(Constants.ClimberConstants.CLIMB_MOTOR);
        talonConfig = new TalonFXConfiguration();
        m_cycleOut = new DutyCycleOut(0);

        talonConfig = new TalonFXConfiguration(); 
        climbMotor.setControl(m_cycleOut); 
    }

    
    public void runClimber(double speed){
        climbMotor.set(speed);
    }

    
    
}
