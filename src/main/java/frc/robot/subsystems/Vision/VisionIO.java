package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO { 

    @AutoLog
    public static class VisionIOInputs
    {
        public Pose2d estimate = new Pose2d();
        public int tagCount = 0;
        public boolean hasEstimatedPose = false; 

        public boolean hasTarget = false; 
        public double yawDeg = 0.0;
        public double pitchDeg = 0.0;
        public double area = 0; 
        public int fiducialID = -1; 

        public double timeStamp = 0; 
        public double latencyMS = 0.0;
    }


    // update set of loggable inputs
    // called periodically in subsystem and upodates vision inputs and pose 2D estimation
    public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {}

    public default PhotonPipelineResult getLatestResult() { return null; }

    public default Matrix<N3, N1> getEstimationSTDDevs(Pose2d estimatePose){return null;}
       
}