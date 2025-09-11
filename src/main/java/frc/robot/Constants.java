package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;


public class Constants {

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public enum Mode {
        REAL, 
        SIM, 
        REPLAY1
    }
    public static class PIDConstants
    {
        // H - Auto Heading PID
        public static final double HKP = 0.10;   
        public static final double HKD = 0.0;   
        public static final double HKI = 0.1;   
        // V - Velocity PID for Roller
        public static final double VKP = 0.00005;
        public static final double VKI = 0;
        public static final double VKD = 0;
        public static final double VKFF = 1.0 / 5676.0;  // ≈ 0.0001762

    }
    public static class OperatorConstants 
    {
        public static int DRIVER_CONTROLLER_PORT = 0;
        public static int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class ClimberConstants
    {
        public static int CLIMB_MOTOR = 10; 
        public static int ClIMB_MOTOR_CURRENT_LIMIT = 60; 
        public static double CLIMB_MOTOR_VOLT_COMP = 12; 
        public static double CLIMBER_SPEED_DOWN = -1;
        public static double CLIMBER_SPEED_UP = 1;
    }
    public static class ArmConstants 
    {
        public static int ARM_MOTOR = 6; 
        public static int ARM_MOTOR_CURRENT_LIMIT = 60; 
        public static final double ARM_MOTOR_VOLTAGE_COMP = 12; 
        public static final double ARM_SPEED = 0.5; 
    }


    public static class RollerConstants
    {
        public static final int ROLLER_MOTOR_ID = 7; 
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 30; 
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 12; 
        public static final double CLIMBER_MOTOR_SPEED_DOWN = 12;
        public static final double CLIMBER_SPEED_DOWN = 0.5; 
        public static final double VELOCITY_RPM = 3000;
    }
    public static class VisionConstants {
        public static final String CAMERA_NAME = "main";
        public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(
            new Translation3d( 
                Units.inchesToMeters(-17.125),
                Units.inchesToMeters(-17.125),
                Units.inchesToMeters(6.825)
            ), 
            new Rotation3d(
                0,
                Units.degreesToRadians(-20),
                Units.degreesToRadians(-135)
            )
            );

        public static final double MAX_AMBIGUITY = 0.2; 
        public static final double MAX_Z_ERROR = 0.5; 

        // allowed uncertainty tolerance in for pose esimtationo
        public static final double LINEAR_STDDEV_BASE = 0.2; //X/Y error 
        public static final double ANGULARSTDDEV_BASE = 0.3; // Rotation error
        
        // PID for strafing back and forth 
        public static final double kXP = 1.0;
        public static final double kXI = 0.0;
        public static final double kXD = 0.0;

        // PID for strafing left and right 
        public static final double kYP = 1.0;
        public static final double kYI = 0.0;
        public static final double kYD = 0.0;
    
        // PID for Rotation
        public static final double kRP = 2.0;
        public static final double kRI = 0.0;
        public static final double kRD = 0.0;
    }
  
}   
