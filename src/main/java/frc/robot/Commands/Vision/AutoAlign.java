package frc.robot.Commands.Vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

public class AutoAlign extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final Vision m_vision;
    private final PhotonCamera m_camera;
    private final PIDController xController = new PIDController(3, 0, 0);
    private final PIDController yController = new PIDController(3, 0, 0);
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(3, 6));
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();



    private final HolonomicDriveController m_holocontroller;
    PhotonPipelineResult results; 
    private Trajectory trajectory;
    private Pose2d targetPose2d;
    private double startTime;
    private Timer timer;
    Pose2d robotPose;
    Rotation2d desiredHeading; 
    private final Pose2d goalPose; 

    private Trajectory generateTrajectory(Pose2d currentRobot, Pose2d targetPose) {
        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                    3, 6
                );
        // An example trajectory to follow.  All units in meters.
        return TrajectoryGenerator.generateTrajectory(
                // Start at the current robot pose
                currentRobot,
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                targetPose,
                config);
    }

    public AutoAlign(CommandSwerveDrivetrain drivetrain, Vision vision, Pose2d targetPose) {
        this.m_drivetrain = drivetrain;
        this.m_vision = vision;
        this.m_camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        this.m_holocontroller = new HolonomicDriveController(xController, yController, thetaController);
        
        this.goalPose = targetPose;
        if (robotPose == null || targetPose == null) {
            cancel();
            return;
        }
        timer = new Timer();    
        
        this.trajectory = generateTrajectory(m_vision.getEstimatedPose(), targetPose);
        addRequirements(drivetrain);
        
        
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_vision.getEstimatedPose();
        if (currentPose == null) {
            // Wait for vision to update; do nothing this cycle
            return;
        }
        Pose2d currentRobotPose = m_vision.getEstimatedPose();
        timer.restart();
        if (currentRobotPose == null || goalPose == null) {
            // No valid pose to start from, cancel command
            cancel();
            return;
        }
    }

    
    @Override
    public void execute() {
        
        Trajectory.State goalState = trajectory.sample(timer.get());

        System.out.println("Trajectory start: " + m_vision.getEstimatedPose() + ", end: " + goalPose);
        System.out.println("Current pose: " + m_vision.getEstimatedPose());
        double elapsed = timer.get();               
        // use timer
        Trajectory.State desiredState = trajectory.sample(elapsed);

        ChassisSpeeds speeds = m_holocontroller.calculate(m_vision.getEstimatedPose(), desiredState, goalPose.getRotation());
        
        m_drivetrain.setControl(applyRobotSpeeds.withSpeeds(speeds));      
    }


    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));  
    }
}