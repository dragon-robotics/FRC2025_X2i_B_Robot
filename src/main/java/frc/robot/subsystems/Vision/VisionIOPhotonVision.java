package frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;


/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    private final PhotonPoseEstimator poseEstimator; 
    private final PhotonCamera camera; 
    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        // configure the pose estimator
        this.poseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT,
         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected(); 
        List<PoseObservation> observations = new ArrayList<>();

        List<Integer> tagIDs = new ArrayList<>(); 
        for (PhotonPipelineResult results : camera.getAllUnreadResults()) {

        if (results.hasTargets()) {
            PhotonTrackedTarget bestTarget = results.getBestTarget();
            inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(results.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(results.getBestTarget().getPitch()));
            

                for (PhotonTrackedTarget targets : results.getTargets()) {
                    tagIDs.add(targets.getFiducialId());
                }    
            
                Optional<EstimatedRobotPose> poseEstimate = poseEstimator.update(results);

                if (poseEstimate.isPresent()) {
                    Pose3d robotPose = poseEstimate.get().estimatedPose;
                    robotPose = new Pose3d(
                        robotPose.getX(),
                        robotPose.getY(), 
                        0.0, // Force Z = 0 (ground level)
                        new Rotation3d(0, 0, robotPose.getRotation().getZ()) // Only keep yaw rotation
                    );

                    double distance = tagDistance(results, robotPose);
                    observations.add(
                        new VisionIO.PoseObservation(
                            results.getTimestampSeconds(),
                             robotPose,
                              bestTarget.getPoseAmbiguity(),
                               results.getTargets().size(),
                                distance,
                                 VisionIO.PoseObservationType.PHOTONVISION)
                    );

                }


                inputs.tagIds = new int[tagIDs.size()];
                for (int i = 0; i < tagIDs.size(); i++) {
                    inputs.tagIds[i] = tagIDs.get(i);
                }
        }   
    }
    inputs.poseObservations = observations.toArray(new VisionIO.PoseObservation[0]);
    inputs.tagIds = tagIDs.stream().mapToInt(Integer::intValue).toArray();

}       
    public double tagDistance(PhotonPipelineResult result, Pose3d robotPose) {
        if(!result.hasTargets()) {
            return 0.0;
        }
        int tagID = result.getBestTarget().getFiducialId(); 
        Optional<Pose3d> tagPose = Constants.VisionConstants.TAG_LAYOUT.getTagPose(tagID);
        
        if (tagPose.isPresent()) {
            // Calculate actual robot-to-tag distance
            return robotPose.getTranslation().getDistance(tagPose.get().getTranslation());
        }
        else {
            return 0.0;
        }
    

    }    
}