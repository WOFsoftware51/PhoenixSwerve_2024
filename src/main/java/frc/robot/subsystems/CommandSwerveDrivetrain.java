package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Function for aiming swerve towards apriltag with limlelight
     * @return targetAngularVelocity: Value within range [-1, 1]. You should multiply the return value by your target max angular velocity
     */
    public double limelight_aim_swerve()
    {    
    double kP = 0.01666;
    double kI = 0.00002;
    double kD = 0.0000125;

    PIDController AimPID = new PIDController(kP, kI, kD);

    double targetAngularVelocity = AimPID.calculate(Global_Variables.tx, 0);// -(tx * kP + kD*txRateOfChange() + kI*txIntegral());

    targetAngularVelocity *= 1;

    return targetAngularVelocity;
    } 
    
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
    public void seedFieldRelativeAngle(double angle) {
        seedFieldRelative(new Pose2d(getState().Pose.getTranslation(), new Rotation2d(angle)));
    }



    /**Configurer for Pathplanner Auton */
    public void configureAuton(){
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->getState().Pose, // Robot pose supplier
            this::seedFieldRelative, // Robot pose supplier
            this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(3, 0.0, 0.0), // Translation PID constants 
                new PIDConstants(100, 0.0, 0.2), // Rotation PID constants
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()), // Default path replanning config. See the API for the options here
            () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            this // Reference to this subsystem to set requirements
        );
    }

    public Command followTrajectoryCommand(String pathString) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathString);
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        return new FollowPathHolonomic(
            path,
            ()->getState().Pose,
            this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                new PIDConstants(10, 0.0, 0.0), // Translation PID constants 
                new PIDConstants(10, 0.0, 0.0), // Rotation PID constants
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()), // Default path replanning config. See the API for the options here
            () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            this // Reference to this subsystem to set requirements
        );
    }



    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        SmartDashboard.putNumber("X axis", getRotation3d().getX()*180/Math.PI);
        SmartDashboard.putNumber("Y axis", getRotation3d().getY()*180/Math.PI);
        SmartDashboard.putNumber("Chassis Speeds", Math.hypot(getCurrentRobotChassisSpeeds().vxMetersPerSecond, getCurrentRobotChassisSpeeds().vyMetersPerSecond));
        SmartDashboard.putNumber("Chassis Speeds X", (getCurrentRobotChassisSpeeds().vxMetersPerSecond));
        SmartDashboard.putNumber("Chassis Speeds Y", getCurrentRobotChassisSpeeds().vyMetersPerSecond);

    
    }
}
