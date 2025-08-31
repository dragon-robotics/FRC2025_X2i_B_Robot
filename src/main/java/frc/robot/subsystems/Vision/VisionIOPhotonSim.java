package frc.robot.subsystems.Vision;
import static edu.wpi.first.units.Units.Rotation;

import java.io.OptionalDataException;
import java.lang.annotation.Target;
import java.nio.channels.NetworkChannel;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionIOPhotonSim implements VisionIO {
    AprilTagFieldLayout tagLayout;
    private final PhotonCamera camera;
    private final VisionSystemSim visionSim; 
    private PhotonCameraSim cameraSim; 
    private final PhotonPoseEstimator photonEstimator; 
    private final SimCameraProperties cameraProp; 

    public VisionIOPhotonSim() {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);

        photonEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);

        cameraProp = new SimCameraProperties(); 
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));        
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);

        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAM);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
        visionSim.update(currentEstimate);
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            inputs.tagCount = result.getTargets().size();

            var bestTarget = result.getBestTarget(); 

            inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(bestTarget.getYaw()),
                Rotation2d.fromDegrees(bestTarget.getPitch()) 
            );
            // update pose estimate 
            photonEstimator.setReferencePose(currentEstimate);
            photonEstimator.update(result).ifPresent(EstimatedRobotPose -> {
                inputs.estimate = EstimatedRobotPose.estimatedPose.toPose2d();
                inputs.timestamp = EstimatedRobotPose.timestampSeconds;
            });
        } else {
            inputs.tagCount = 0; 
            inputs.tagIds = new int[0];
            inputs.latestTargetObservation = new TargetObservation(
                new Rotation2d(), 
                new Rotation2d()
            );
        }
        
    
    }

    public Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonEstimator.update(camera.getLatestResult());
    }

    


    

}
