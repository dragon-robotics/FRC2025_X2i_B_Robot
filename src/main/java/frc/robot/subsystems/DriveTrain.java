package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.OptionalDataException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.swing.plaf.OptionPaneUI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
public class DriveTrain extends SubsystemBase  {
    private final CommandSwerveDrivetrain m_swerve; 
    private static double maxSpeed; 
    private static double maxAngularRate;
    private Optional<Rotation2d> currentHeading; // Add this field

    private double rotationLastTriggered = 0;
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.FieldCentricFacingAngle driveMaintainHeading;

    public DriveTrain(CommandSwerveDrivetrain swerve) {
        // Initialize variables
        m_swerve = swerve;
        maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
        currentHeading = Optional.empty();
        drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        driveMaintainHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        driveMaintainHeading.HeadingController.setPID(
            Constants.DriveTrainConstants.HEADING_kP,
            Constants.DriveTrainConstants.HEADING_kI,
            Constants.DriveTrainConstants.HEADING_kD
        );
        driveMaintainHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        driveMaintainHeading.HeadingController.setTolerance(Constants.DriveTrainConstants.HEADING_TOLERANCE);
    


}

public Command driveCommand(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup
) {
    return new RunCommand(
        () -> 
        {      double rawTranslation = translationSup.getAsDouble();
            double rawStrafe = strafeSup.getAsDouble();
            double rawRotation = rotationSup.getAsDouble();
            
            // Use constant for deadband
            double translation = MathUtil.applyDeadband(rawTranslation, Constants.DriveTrainConstants.DEADBAND);
            double strafe = MathUtil.applyDeadband(rawStrafe, Constants.DriveTrainConstants.DEADBAND);
            double rotation = MathUtil.applyDeadband(rawRotation, Constants.DriveTrainConstants.DEADBAND);
            
            // Apply input shaping
            translation = Math.copySign(
                Math.pow(Math.abs(translation), Constants.DriveTrainConstants.TRANSLATION_EXPONENT), 
                translation
            );
            strafe = Math.copySign(
                Math.pow(Math.abs(strafe), Constants.DriveTrainConstants.TRANSLATION_EXPONENT), 
                strafe
            );
            rotation = Math.copySign(
                Math.pow(Math.abs(rotation), Constants.DriveTrainConstants.ROTATION_EXPONENT), 
                rotation
            );
            translation *= maxSpeed * Constants.DriveTrainConstants.TRANSLATION_SCALE;
            strafe *= maxSpeed * Constants.DriveTrainConstants.TRANSLATION_SCALE;
            rotation *= maxAngularRate * Constants.DriveTrainConstants.ROTATION_SCALE;
    

            boolean rotationTriggered = Math.abs(rawRotation) > Constants.DriveTrainConstants.DEADBAND; 

            if (rotationTriggered) {
                m_swerve.setControl(drive
                .withVelocityX(translation)
                .withVelocityY(strafe)
                .withRotationalRate(rotation));
                currentHeading = Optional.empty();
            } else {
                if (currentHeading.isEmpty()) {
                    currentHeading = Optional.of(m_swerve.getState().Pose.getRotation());
                }
                // ADD: Clamp the heading hold rotation to prevent aggressive corrections
                double headingCorrection = maintainOdometry(m_swerve.getState().Pose.getRotation().getRadians());
                headingCorrection = MathUtil.clamp(headingCorrection, -2.0, 2.0); // Limit to 2 rad/s

                m_swerve.setControl(drive
                .withVelocityX(translation)
                .withVelocityY(strafe)
                .withRotationalRate(headingCorrection));

            }
        }, m_swerve
        );
}










    public double maintainOdometry(double currentHeadingRadians) {
        if (currentHeading.isEmpty()) {
            currentHeading = Optional.of(new Rotation2d(currentHeadingRadians));
        }

        double targetHeading = currentHeading.get().getRadians();

        double headingError = targetHeading - currentHeadingRadians;

        while (headingError > Math.PI) {
            headingError -= 2 * Math.PI;

        }
        while ( headingError < -Math.PI) {
            headingError += 2 * Math.PI;
        }

        return headingError * Constants.DriveTrainConstants.HEADING_kP;
    }

    public void resetHeading() {
        currentHeading = Optional.empty();
    }
    
    public void seedFieldCentric() {
        m_swerve.seedFieldCentric();
        currentHeading = Optional.empty(); // Reset heading hold too
        Timer.delay(0.1);

    }
    public CommandSwerveDrivetrain getSwerve()
    {
        return m_swerve; 
    }
}