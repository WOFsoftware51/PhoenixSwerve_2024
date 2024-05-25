package frc.robot.commands_Auton;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.field.AprilTag;
import frc.robot.field.Field;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class AutonSwerveAim extends Command {    
    private CommandSwerveDrivetrain s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private Rotation2d rotationTarget = new Rotation2d();
    private AprilTag targetAprilTag = Field.Blue.kAprilTag7;

    private boolean endCommand = false;
    private int count = 0;
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    

    public AutonSwerveAim(CommandSwerveDrivetrain Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.s_Swerve = Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
    }
    @Override
    public void initialize() {
      /* Get Values, Deadband*/
      endCommand = false;
      count = 0;
      if(DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get() == Alliance.Blue){
          targetAprilTag = Field.Blue.kAprilTag7;
        }
        if(DriverStation.getAlliance().get() == Alliance.Red){
          targetAprilTag = Field.Red.kAprilTag4;
        }
      }
  }

    @Override
    public void execute() {

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    Translation2d robotVisionBaesdPosition = Global_Variables.visionPoseEstimate2d.pose.getTranslation();

    if(Global_Variables.tv == 1){

      if(Global_Variables.tx < 4.0 && Global_Variables.tx > -4.0 && count > 15){
        endCommand = true;
      }
      else{
        rotationTarget = 
          new Rotation2d(360 * (Math.PI/180))
            .minus(new Rotation2d(robotVisionBaesdPosition.getX() - targetAprilTag.getPosition().getX(), robotVisionBaesdPosition.getY() - targetAprilTag.getPosition().getY()));
      }
      count++;
    }
    else{
      // rotationVal = s_Swerve.gotoDefaultGyroVal(); 
    }
    
    /* Drive */
      s_Swerve.applyRequest(() -> drive.withVelocityX(translationVal * TunerConstants.kSpeedAt12VoltsMps).withVelocityY(strafeVal * TunerConstants.kSpeedAt12VoltsMps).withTargetDirection(rotationTarget));
    }

    
    @Override
    public void end(boolean interrupted) {
      s_Swerve.applyRequest(() -> drive.withVelocityX(0) .withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        return endCommand;
    }

}