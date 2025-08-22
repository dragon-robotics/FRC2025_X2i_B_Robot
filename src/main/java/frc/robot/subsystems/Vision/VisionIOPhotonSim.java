package frc.robot.subsystems.Vision;
import org.photonvision.PhotonCamera;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;


public class VisionIOPhotonSim implements VisionIO {
    
    // add photon camera and photon pose estimator fields
    public VisionIOPhotonSim(String CameraName)
    {
        VisionSystemSim visionSim = new VisionSystemSim(CameraName);
        // TargetModel.
        VisionSystemSim sim;

    }

    
    public void updateInputs(VisionIOInputs inputs) {
        // TODO: 
        // 1. Get latest PhotonPipelineResult
        // 2. If has targets, fill yaw, pitch, area, fiducialId
        // 3. Always update latency, timestamp
        // 4. If using AprilTags, fill estimate
    }
}
