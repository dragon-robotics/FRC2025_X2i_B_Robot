package frc.robot.subsystems.Vision;

import java.util.function.Supplier;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonSim implements VisionIO {
    private final PhotonCamera camera;
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator photonEstimator;
    private final Supplier<Pose2d> poseSupplier;

    public VisionIOPhotonSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // Initialize PhotonCamera
        camera = new PhotonCamera(name);

        // Initialize VisionSystemSim and add AprilTags
        visionSim = new VisionSystemSim(name);
        visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);

        // Initialize PhotonPoseEstimator
        photonEstimator = new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );

        // Configure SimCameraProperties
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        // Initialize PhotonCameraSim
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Add the simulated camera to the vision system
        visionSim.addCamera(cameraSim, robotToCamera);
        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update the vision simulation with the current robot pose
        visionSim.update(poseSupplier.get());

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

            // Process results (similar to PhotonVision implementation)
            var photonResults = photonEstimator.update(result);
            if (photonResults.isPresent()) {
                var estimatedPose = photonResults.get();
                
                // Add tag IDs
                for (var target : result.targets) {
                    tagIds.add((short) target.fiducialId);
                }

                // Calculate average tag distance
                double totalTagDistance = result.targets.stream()
                    .mapToDouble(target -> target.bestCameraToTarget.getTranslation().getNorm())
                    .sum();

                // Add observation
                poseObservations.add(new PoseObservation(
                    result.getTimestampSeconds(),
                    estimatedPose.estimatedPose,
                    0.0, // Sim doesn't have ambiguity
                    result.targets.size(),
                    totalTagDistance / result.targets.size(),
                    PoseObservationType.PHOTONVISION
                ));
            }
        }

        // Save pose observations and tag IDs
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
        inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
    }
}
