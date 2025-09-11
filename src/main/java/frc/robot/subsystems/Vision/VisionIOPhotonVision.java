package frc.robot.subsystems.Vision;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.StackWalker.Option;
import java.sql.ResultSet;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
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
    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected(); 
        List<PoseObservation> observations = new ArrayList<>();
        PhotonPipelineResult results = camera.getLatestResult();

        List<Integer> tagIDs = new ArrayList<>(); 
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
                    Pose2d robotPose = poseEstimate.get().estimatedPose.toPose2d();

                    double distance = Tagdistance(results, robotPose);
                    observations.add(
                        new VisionIO.PoseObservation(
                            distance,
                             robotPose,
                              bestTarget.getPoseAmbiguity(),
                               results.getTargets().size(),
                                distance,
                                 VisionIO.PoseObservationType.MEGATAG_2)
                    );

                }

                inputs.poseObservations = observations.toArray(new VisionIO.PoseObservation[0]);

                inputs.tagIds = new int[tagIDs.size()];
                for (int i = 0; i < tagIDs.size(); i++) {
                    inputs.tagIds[i] = tagIDs.get(i);
                }
        }   
        
            


        

    
}       
    public double Tagdistance(PhotonPipelineResult result, Pose3d robotPose) {
        int tagID = result.getBestTarget().getFiducialId(); 
        Optional<Pose3d> tagPose = Constants.VisionConstants.TAG_LAYOUT.getTagPose(tagID);
        
        if (tagPose.isPresent()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(Constants.VisionConstants.ROBOT_TO_CAM.getY(), 
            tagPose.get().getZ(),
            Constants.VisionConstants.ROBOT_TO_CAM.getRotation().getY(), // camera pitch in radians
            result.getBestTarget().getPitch());
            return distance; 
        }
        else {
            return 0;
        }
    }    
}