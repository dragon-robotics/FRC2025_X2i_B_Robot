package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PIDConstants; // assuming you have PID constants


public class ArmSubsystem extends SubsystemBase
{

    private final SparkMax armMotor; 
    private final AbsoluteEncoder armEncoder; 
    private final SparkClosedLoopController armController; 
    private final double targetAngle = 45; 

    public ArmSubsystem()
    {
        armMotor = new SparkMax(ArmConstants.ARM_MOTOR, MotorType.kBrushless);
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armEncoder = armMotor.getAbsoluteEncoder(); 
        armController = armMotor.getClosedLoopController();

        armConfig.voltageCompensation(10)
        .smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT).
        idleMode(IdleMode.kBrake);
        
    }

    public double getArmAngle()
    {
        return armEncoder.getPosition();
    }

    public void pivotArm()
    {
        armController.setReference(targetAngle, SparkMax.ControlType.kPosition);
    }

    public void stop()
    {
        armMotor.stopMotor();
    }

    public double getCurrent()
    {
        return armMotor.getOutputCurrent();
    }
}
