package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    // Current robot mode
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
        REAL,
        SIM,
        REPLAY1
    }

    // PID Constants
    public static class PIDConstants {
        // Auto Heading PID
        public static final double HKP = 0.10;   
        public static final double HKD = 0.0;   
        public static final double HKI = 0.1;   

        // Velocity PID for Roller
        public static final double VKP = 0.00005;
        public static final double VKI = 0;
        public static final double VKD = 0;
        public static final double VKFF = 1.0 / 5676.0;  // ≈ 0.0001762
    }

    // Controller ports
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    // Climber constants
    public static class ClimberConstants {
        public static final int CLIMB_MOTOR = 10; 
        public static final int CLIMB_MOTOR_CURRENT_LIMIT = 60; 
        public static final double CLIMB_MOTOR_VOLT_COMP = 12; 
        public static final double CLIMBER_SPEED_DOWN = -1;
        public static final double CLIMBER_SPEED_UP = 1;
    }

    // Arm constants
    public static class ArmConstants {
        public static final int ARM_MOTOR = 6; 
        public static final int ARM_MOTOR_CURRENT_LIMIT = 60; 
        public static final double ARM_MOTOR_VOLTAGE_COMP = 12; 
        public static final double ARM_SPEED = 0.5; 
    }

    // Roller constants
    public static class RollerConstants {
        public static final int ROLLER_MOTOR_ID = 7; 
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 30; 
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 12; 
        public static final double VELOCITY_RPM = 3000;
    }

    // Vision constants
    public static class VisionConstants {
        public static final String APTAG_POSE_EST_CAM_FL_POS_NAME = "Front-Left Camera";
        public static final String APTAG_POSE_EST_CAM_FR_POS_NAME = "Front-Right Camera";

        // Default field layout for AprilTags
        public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        public static final int[] REEF_TAG_IDS = {5, 6, 7, 8, 9, 10, 11, 12}; // 2025 reef tags
        // Transform from robot origin to camera
        public static final Transform3d APTAG_POSE_EST_CAM_FL_POS =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(17.125),
                Units.inchesToMeters(17.125),
                Units.inchesToMeters(6.825)),
            new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(45)));

    // Front-Right Camera: Mounted at front-right corner, pointing outward at -30
    // degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FR_POS =
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-45)));

        // Maximum allowed ambiguity / error for pose acceptance
        public static final double MAX_AMBIGUITY = 0.2; 
        public static final double MAX_Z_ERROR = 0.5; 

        // Allowed uncertainty tolerance in pose estimation
        public static final double LINEAR_STDDEV_BASE = 0.2; // meters
        public static final double ANGULAR_STDDEV_BASE = 0.3; // radians

        // Factors for specific observation types
        public static final double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5;
        public static final double ANGULAR_STDDEV_MEGATAG2_FACTOR = 0.5;

        // Camera-specific standard deviation factors
        public static final double[] CAMERA_STDDEV_FACTORS = {1.0, 1.0}; // Example for two cameras

        // PID for strafing (X/Y) and rotation
        public static final double kXP = 1.0;
        public static final double kXI = 0.0;
        public static final double kXD = 0.0;

        public static final double kYP = 1.0;
        public static final double kYI = 0.0;
        public static final double kYD = 0.0;

        public static final double kRP = 2.0;
        public static final double kRI = 0.0;
        public static final double kRD = 0.0;

        // Debugging constants
        public static final boolean ENABLE_POSE_LOGGING = true;
        public static final boolean ENABLE_TAG_LOGGING = true;
        
    }
}
