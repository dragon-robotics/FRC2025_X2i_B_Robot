package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.PIDConstants;

public class RollerSubsystem extends SubsystemBase{
    private static SparkMax RollerMotor; 
    private static SparkClosedLoopController roller_Controller; 
    public RollerSubsystem()
    {
        RollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        roller_Controller = RollerMotor.getClosedLoopController();


        SparkMaxConfig rollerConfig = new SparkMaxConfig();

         rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP)
         .smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT)
         .idleMode(IdleMode.kBrake);

         rollerConfig.closedLoop.p(PIDConstants.VKP)
         .i(0, ClosedLoopSlot.kSlot0)
         .p(0, ClosedLoopSlot.kSlot0)
         .velocityFF(PIDConstants.VKFF, ClosedLoopSlot.kSlot1);
    
        RollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runRoller(double velocity)
    {
        roller_Controller.setReference(velocity, ControlType.kVelocity);
    }

    public void stopRoller() {
        RollerMotor.set(0);
    }
        
}
