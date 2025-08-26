// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Commands.ClimberDownCommand;
import frc.robot.Commands.ClimberUpCommand;
import frc.robot.Commands.RollerIntakeCommand;
import frc.robot.Commands.RollerScoreCommand;
import frc.robot.Commands.RotateArmCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RollerSubsystem;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)*.25; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond)*.3; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController m_operatorController = new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser; 


    public final ArmSubsystem m_arm = new ArmSubsystem();
    public final ClimberSubsystem m_climber = new ClimberSubsystem();
    public final RollerSubsystem m_roller = new RollerSubsystem();


    private Double targetHeading; // Use Double for nullability
    PIDController headingPID = new PIDController(Constants.PIDConstants.HKP, Constants.PIDConstants.HKI, Constants.PIDConstants.HKD);

    public RobotContainer() {

        NamedCommands.registerCommand("RollerIntake", new RollerIntakeCommand(m_roller, Constants.RollerConstants.VELOCITY_RPM));
        NamedCommands.registerCommand("RollerScore", new RollerScoreCommand(m_roller, Constants.RollerConstants.VELOCITY_RPM));
        NamedCommands.registerCommand("RotateArm", new RotateArmCommand(m_arm));

        autoChooser = AutoBuilder.buildAutoChooser("SimpleAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);


        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        configureBindings();

    }


    private void configureBindings() {
       

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // // Drivetrain will execute this command periodically
            // drivetrain.applyRequest(() ->
            // {
            //     double translate = -joystick.getLeftY();
            //     double strafe = -joystick.getLeftX();
            //     double rotate = joystick.getRightX();

            //     boolean joystickActive = Math.abs(joystick.getRightX()) > 0.1;
            //     double currentHeading = drivetrain.getHeading();
            //     double rotationInput;
            //     if (joystickActive) {
            //         rotationInput = -joystick.getRightX() * MaxAngularRate;
            //     }
            //     else {
            //         rotationInput = maintainOdometry(joystick, currentHeading);
            //     }
        
            //     return 
            //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         .withVelocityY(-joystick.getLeftX() * MaxSpeed)  // Drive left with negative X (left)
            //         .withRotationalRate(rotationInput);
                        // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );                                                                                                                                                                                                                                                                                          

        // Idle while the robot is disabled. This ensures the configured                              
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        m_operatorController.leftTrigger(0.2).whileTrue(new RollerIntakeCommand(m_roller, -Constants.RollerConstants.VELOCITY_RPM));
        m_operatorController.rightTrigger(0.2).whileTrue(new RollerScoreCommand(m_roller, Constants.RollerConstants.VELOCITY_RPM));

        m_operatorController.b().onTrue(new RotateArmCommand(m_arm));
        m_operatorController.pov(0).whileTrue(new ClimberUpCommand(m_climber));
        m_operatorController.pov(180).whileTrue(new ClimberDownCommand(m_climber));        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.


        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }




 

    public double maintainOdometry(CommandXboxController joystick, double currentHeading)
    {
        if (targetHeading == null) // Check if targetHeading is not set
        {
            targetHeading = currentHeading;
        }            
        double rotationCorrection = headingPID.calculate(currentHeading, targetHeading);
        double radiansPersecond = Math.toRadians(rotationCorrection);
        return radiansPersecond;   
    }

}
