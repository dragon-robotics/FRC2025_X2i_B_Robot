// package frc.robot.Commands.Vision;

// import com.ctre.phoenix6.swerve.SwerveRequest;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.HolonomicDriveController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Constants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Vision.Vision;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;

// import java.lang.StackWalker.Option;
// import java.util.List;
// import java.util.Optional;

// public class AutoAlign extends Command {

//     private final CommandSwerveDrivetrain m_drivetrain;
//     private final PIDController xController = new PIDController(1, 0, 0);
//     private final PIDController yController = new PIDController(1, 0, 0);
//     private final ProfiledPIDController thetaController =
//             new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3, 6));
//     private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();



//     PhotonPipelineResult results; 
//     Pose2d robotPose;
//     Rotation2d desiredHeading; 
//     private final Pose2d goalPose; 
//     private final Vision m_vision; 

//     Pose2d currentRobotPose;
//     public AutoAlign(CommandSwerveDrivetrain drivetrain, Pose2d targetPose, Vision vision) {
//         this.m_drivetrain = drivetrain;
//         this.goalPose = targetPose;
//         this.m_vision = vision;
//         addRequirements(drivetrain);        
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     }

//     @Override
//     public void initialize() {

//         currentRobotPose = m_vision.getEstimatedPose();
//         ChassisSpeeds speeds = m_drivetrain.getState().Speeds;
        
//         ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//             speeds, currentRobotPose.getRotation()
//         );
//         xController.reset();
//         yController.reset();
//         thetaController.reset(
//             currentRobotPose.getRotation().getRadians(),fieldRelativeSpeeds.omegaRadiansPerSecond
//         );
         

//     }

    
//     @Override
//     public void execute() {

//             // Add null check for goalPose
//         if (goalPose == null) {
//         System.err.println("AutoAlign: goalPose is null - cancelling command");
//         cancel();
//         return;
//         }

//         currentRobotPose = m_vision.getEstimatedPose();

//         if (currentRobotPose == null) {
//         currentRobotPose = m_drivetrain.getPose2d();
//         if (currentRobotPose == null) {
//             System.err.println("AutoAlign: No robot pose available - cancelling");
//             cancel();
//             return;
//         }
//         }
//         currentRobotPose = m_vision.getEstimatedPose();

//         double xSpeed = xController.calculate(
//             currentRobotPose.getX(), goalPose.getX()
//         );
//         double ySpeed = yController.calculate(
//             currentRobotPose.getY(), goalPose.getY()
//         );

      
//         double thetaSpeed = thetaController.calculate(
//             currentRobotPose.getRotation().getRadians(), goalPose.getRotation().getRadians()
//         );
        
//         xSpeed = MathUtil.clamp(xSpeed, -3, 3);
//         ySpeed = MathUtil.clamp(ySpeed, -3, 3);

//         thetaSpeed = MathUtil.clamp(thetaSpeed, -Math.PI, Math.PI);

//         SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
//             .withVelocityX(xSpeed)
//             .withVelocityY(ySpeed)
//             .withRotationalRate(thetaSpeed);

//         m_drivetrain.setControl(fieldCentricRequest);
//     }


//     @Override
//     public boolean isFinished() {
//         return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
//     }
//     @Override
//     public void end(boolean interrupted) {
        
//         m_drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
//     }
// }