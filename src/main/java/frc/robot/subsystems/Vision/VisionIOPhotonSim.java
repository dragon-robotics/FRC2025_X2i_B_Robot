package frc.robot.subsystems.Vision;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Constants.VisionConstants;

public class VisionIOPhotonSim implements VisionIO {
    AprilTagFieldLayout tagLayout;
    private final PhotonCamera camera;
    private final VisionSystemSim visionSim; 
    private PhotonCameraSim cameraSim; 
    private final PhotonPoseEstimator photonEstimator; 
    private final SimCameraProperties cameraProp; 
    private final Supplier<Pose2d> poseSupplier;
    public VisionIOPhotonSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier; 
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
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
    }
}
