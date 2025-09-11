package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO { 
    @AutoLog
    public static class VisionIOInputs {
        boolean connected = false; 
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
        public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    }
    // data container for what each camera sees
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double tagDistance,
        PoseObservationType type) {}

    // list of pipelines we can use
    public static enum PoseObservationType {
        MEGATAG_2,
        PHOTONVISION        
    }
    // update set of loggable inputs
    // called periodically in subsystem and upodates vision inputs and pose 2D estimation
    public default void updateInputs(VisionIOInputs inputs) {}

    // gets the latest offset in pitch and yaw of the target observed 
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}
}