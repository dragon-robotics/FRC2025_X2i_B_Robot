package frc.robot.subsystems.Vision;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionIO.PoseObservation;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

import java.lang.StackWalker.Option;
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
    private final VisionIOInputs[] inputs; 
    // intializes a list that can tell us how many cameras have disconnected
    private final Alert[] diconnectedAlerts;  
    
    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        this.inputs = new VisionIOInputs[io.length];
        this.diconnectedAlerts = new Alert[io.length];
        
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputs();
            diconnectedAlerts[i] = 
                new Alert(
                   "Camera"  + Integer.toString(i) + "is disconnected", AlertType.kWarning);
        }
    }

    

    @Override 
    public void periodic() {
        // TODO: Get camera data over time, log it, process, and filter it
        
        for(int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
        }
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();
        
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            diconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
            
            List<Pose3d> tagPoses = new LinkedList<>(); 
            List<Pose3d> robotPoses = new LinkedList<>(); 
            List<Pose3d> robotPosesAccepted = new LinkedList<>(); 
            List<Pose3d> robotPosesRejected = new LinkedList<>(); 
            // get april tag position of every april tag each camera identifies
            for (int tagID : inputs[cameraIndex].tagIds) {
                var tagPose = Constants.VisionConstants.TAG_LAYOUT.getTagPose(tagID);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }
            

            for(var observation: inputs[cameraIndex].poseObservations) {
                boolean rejectpose = 
                (observation.tagCount() == 0 || 
                observation.tagCount() == 1 && observation.ambiguity() > Constants.VisionConstants.MAX_AMBIGUITY
                || Math.abs(observation.pose().getZ()) > Constants.VisionConstants.MAX_Z_ERROR
                
                // pose must be inside of the field 
                || observation.pose().getX() > Constants.VisionConstants.TAG_LAYOUT.getFieldLength()
                || observation.pose().getX() < 0.0
                || observation.pose().getY() > Constants.VisionConstants.TAG_LAYOUT.getFieldWidth()
                || observation.pose().getY() < 0.0
                );

                
                robotPoses.add(observation.pose());
                if (rejectpose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }
                if (rejectpose) {
                    continue;
                }
                
                latestAcceptedPose = observation.pose().toPose2d();
                // double distance = observation.averageTagDistance(); 
                // int tagCount = observation.tagCount();
                double timeStampSeconds = observation.timestamp();

                // double stdDevFactor = Math.pow(distance, 2.0) / tagCount; 

                double linearStdDev = Constants.VisionConstants.LINEAR_STDDEV_BASE;
                double angularStdDev = Constants.VisionConstants.ANGULARSTDDEV_BASE; 

                Matrix<N3, N1> stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
                consumer.accept(
                    observation.pose().toPose2d(),
                    timeStampSeconds, stdDevs);


                // log all positiions
                Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[(robotPoses.size())]));
                Logger.recordOutput("Vision/Camera" + cameraIndex +  "/RobotPosesAccepted", robotPoses.toArray(new Pose3d[robotPosesAccepted.size()]));
                Logger.recordOutput("Vision/Camera" + cameraIndex +  "/RobotPosesRejected", robotPoses.toArray(new Pose3d[robotPosesRejected.size()]));
                Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
           
                
                allTagPoses.addAll(tagPoses);
                allRobotPoses.addAll(robotPoses);
                allRobotPosesAccepted.addAll(robotPosesAccepted);
                allRobotPosesRejected.addAll(robotPosesRejected);
            }
            }
        
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
            PoseObservation bestObservation = null;

        for (VisionIOInputs cameraInputs : inputs) {
            for (PoseObservation obs : cameraInputs.poseObservations) {
                boolean valid = obs.tagCount() > 0f
                && obs.ambiguity() < Constants.VisionConstants.MAX_AMBIGUITY
                && obs.pose().getX() >= 0 && obs.pose().getX() <= VisionConstants.TAG_LAYOUT.getFieldLength()
                && obs.pose().getY() >= 0 && obs.pose().getY() <= VisionConstants.TAG_LAYOUT.getFieldWidth();
           
                if (!valid) {
                    continue;
                }
                if (bestObservation == null || obs.ambiguity() < bestObservation.ambiguity())
                {
                    bestObservation = obs;
                }
            }
        }
        if (bestObservation == null) {
            return null;
        }
        
        return bestObservation.pose();

    }
}