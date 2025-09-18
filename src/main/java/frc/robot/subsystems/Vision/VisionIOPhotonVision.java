package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    private final PhotonPoseEstimator poseEstimator;
    private final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
        // Configure the pose estimator
        this.poseEstimator = new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch())
                );
            }

            // Process multitag results
            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(poseEstimator.getRobotToCameraTransform().inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = result.targets.stream()
                    .mapToDouble(target -> target.bestCameraToTarget.getTranslation().getNorm())
                    .sum();

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(new PoseObservation(
                    result.getTimestampSeconds(),
                    robotPose,
                    multitagResult.estimatedPose.ambiguity,
                    multitagResult.fiducialIDsUsed.size(),
                    totalTagDistance / result.targets.size(),
                    PoseObservationType.PHOTONVISION
                ));
            } else if (!result.targets.isEmpty()) {
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                        tagPose.get().getTranslation(),
                        tagPose.get().getRotation()
                    );
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(poseEstimator.getRobotToCameraTransform().inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(),
                        robotPose,
                        target.poseAmbiguity,
                        1,
                        cameraToTarget.getTranslation().getNorm(),
                        PoseObservationType.PHOTONVISION
                    ));
                }
            }
        }

        // Save pose observations and tag IDs
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
    }

    public double calculateTagDistance(PhotonPipelineResult result, Pose3d robotPose) {
        int tagID = result.getBestTarget().getFiducialId();
        var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tagID);

        if (tagPose.isPresent()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.ROBOT_TO_CAM.getY(),
                tagPose.get().getZ(),
                VisionConstants.ROBOT_TO_CAM.getRotation().getY(),
                result.getBestTarget().getPitch()
            );
        } else {
            return 0;
        }
    }
}