package frc.robot.subsystems.Vision;

import java.lang.annotation.Target;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;


public interface VisionIO { 

    @AutoLog
    public static class VisionIOInputs {
        public Pose2d estimate = new Pose2d(); 
        public int tagCount = 0; 
        public double timestamp = 0; 
        public boolean connected = false; 
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public int[] tagIds = new int[0];
    }

    
    // update set of loggable inputs
    // called periodically in subsystem and upodates vision inputs and pose 2D estimation
    public default void updateInputs(VisionIOInputs inputs, Pose2d estimate) {}

    public default PhotonPipelineResult getLatestResult() { return null; }

    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    public default Matrix<N3, N1> getEstimationSTDDevs(Pose2d estimatePose){return null;}

    public default Optional<EstimatedRobotPose> getEstimatedGlobalPose() { return null; }

}