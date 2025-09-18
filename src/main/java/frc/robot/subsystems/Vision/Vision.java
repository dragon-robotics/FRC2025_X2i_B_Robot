package frc.robot.subsystems.Vision;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionIO.PoseObservation;
import frc.robot.subsystems.Vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase{
    private final VisionConsumer consumer;
    Pose2d latestAcceptedPose = null; 
    // initializes a list of cameras being used 
    private final VisionIO[] io;
    // initializes a list of visionIOI basically data containers for our cameras
    private final VisionIOInputsAutoLogged[] inputs;
    // intializes a list that can tell us how many cameras have disconnected
    private final Alert[] diconnectedAlerts;  
    
    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.diconnectedAlerts = new Alert[io.length];
        
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
        for (int i = 0; i < inputs.length; i++) {
            diconnectedAlerts[i] = 
                new Alert(
                   "Camera"  + Integer.toString(i) + "is disconnected", AlertType.kWarning);
        }
    }

    

    @Override 
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);       
            Logger.recordOutput("Vision/Test/ForcedData", true);

        }
        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            diconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = Constants.VisionConstants.TAG_LAYOUT.getTagPose(tagId);
                tagPose.ifPresent(tagPoses::add);
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > Constants.VisionConstants.MAX_AMBIGUITY)
                || Math.abs(observation.pose().getZ()) > Constants.VisionConstants.MAX_Z_ERROR
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > Constants.VisionConstants.TAG_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > Constants.VisionConstants.TAG_LAYOUT.getFieldWidth()
                || observation.tagDistance() > 5.0; // Reject if tags are too far away    
                // Create floor-constrained pose
                var originalPose = observation.pose();
                var floorConstrainedPose = new Pose2d(
                    originalPose.getTranslation().toTranslation2d(), // Properly extract 2D translation
                    originalPose.getRotation().toRotation2d()        // Properly extract 2D rotation
                );               
                var floorConstrainedPose3d = new Pose3d(
                    new Translation3d(
                        floorConstrainedPose.getX(),
                        floorConstrainedPose.getY(),
                        0.0  // Z should be 0 for floor-constrained pose
                    ),
                    new Rotation3d(0, 0, floorConstrainedPose.getRotation().getRadians())
                );
                robotPoses.add(floorConstrainedPose3d);

                if (rejectPose) {
                    robotPosesRejected.add(floorConstrainedPose3d);
                    continue; // Skip to next observation
                }
                robotPosesAccepted.add(floorConstrainedPose3d);


                latestAcceptedPose = floorConstrainedPose;

                double stdDevFactor = Math.pow(observation.tagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = Constants.VisionConstants.LINEAR_STDDEV_BASE * stdDevFactor;
                double angularStdDev = Constants.VisionConstants.ANGULAR_STDDEV_BASE * stdDevFactor;

                // if (cameraIndex < Constants.VisionConstants.CAMERA_STDDEV_FACTORS.length) {
                //     linearStdDev *= Constants.VisionConstants.CAMERA_STDDEV_FACTORS[cameraIndex];
                //     angularStdDev *= Constants.VisionConstants.CAMERA_STDDEV_FACTORS[cameraIndex];
                // }
                consumer.accept(
                        floorConstrainedPose,
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }
    
    @FunctionalInterface
    public static interface VisionConsumer {
        void accept(
            Pose2d visionRobotPose,
            double timeStampSeconds, 
            Matrix<N3, N1> VisionMeasurementsStdDevs
        );
    }

    public Pose2d getEstimatedPose() {
        return latestAcceptedPose;
    }


  public Pose3d getBestTagPose3d() {
    List<PoseObservation> validObservations = new ArrayList<>();

    for (VisionIOInputs cameraInputs : inputs) {
        for (PoseObservation obs : cameraInputs.poseObservations) {
            boolean valid = obs.tagCount() > 0
                    && obs.ambiguity() < Constants.VisionConstants.MAX_AMBIGUITY
                    && obs.pose().getX() >= 0
                    && obs.pose().getX() <= Constants.VisionConstants.TAG_LAYOUT.getFieldLength()
                    && obs.pose().getY() >= 0
                    && obs.pose().getY() <= Constants.VisionConstants.TAG_LAYOUT.getFieldWidth();

            if (valid) {
                validObservations.add(obs);
            }
        }
    }

    if (validObservations.isEmpty()) {
        return null; // no valid tags
    }

    // Average X/Y/Z
    double avgX = validObservations.stream().mapToDouble(o -> o.pose().getX()).average().orElse(0);
    double avgY = validObservations.stream().mapToDouble(o -> o.pose().getY()).average().orElse(0);
    double avgZ = validObservations.stream().mapToDouble(o -> o.pose().getZ()).average().orElse(0);

    // Average rotation by converting to unit vectors
    double sumSin = 0;
    double sumCos = 0;
    for (PoseObservation obs : validObservations) {
        Rotation3d rot = obs.pose().getRotation();
        double yaw = rot.getZ(); // yaw around vertical axis
        sumSin += Math.sin(yaw);
        sumCos += Math.cos(yaw);
    }
    double avgYaw = Math.atan2(sumSin / validObservations.size(), sumCos / validObservations.size());

    Rotation3d avgRotation = new Rotation3d(0, 0, avgYaw);

    return new Pose3d(avgX, avgY, avgZ, avgRotation);
}

}
