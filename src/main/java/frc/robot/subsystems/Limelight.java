// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private double tx = 0.0;
    private double ty = 0.0;
    private double tv = 0.0;
    private double distanceY = 0.0;

    /** Creates a new Limelight. */
    public Limelight() {

    }
    
    private double getDistanceYFixed(){
        return ((0.0)*Math.pow(distanceY, 0) + 0.0);
    }
    private PoseEstimate getVisionPoseEstimate2d(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }





  @Override
  public void periodic() {
    
    Global_Variables.tx = tx;
    Global_Variables.ty = ty;
    Global_Variables.tv = tv;
    Global_Variables.distanceY = distanceY;
    Global_Variables.distanceYFixed = getDistanceYFixed();
    Global_Variables.visionPoseEstimate2d = getVisionPoseEstimate2d();

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("tv", Global_Variables.tv);
    SmartDashboard.putNumber("ty", Global_Variables.ty);
    SmartDashboard.putNumber("Vision Pose X", getVisionPoseEstimate2d().pose.getX());
    SmartDashboard.putNumber("Vision Pose Y", getVisionPoseEstimate2d().pose.getY());

    tv = table.getEntry("tv").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    tx = table.getEntry("tx").getDouble(0);

    distanceY = (Constants.APRIL_TAG_HEIGHT-Constants.LIMELIGHT_HEIGHT)/(Math.tan(Math.toRadians(Constants.LIMELIGHT_ANGLE+ty)));
}
}
