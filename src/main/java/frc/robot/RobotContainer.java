// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Blue_Leave_Zone;
import frc.robot.autos.Shoot_Only_Auto;
import frc.robot.autos.TestAuton;
import frc.robot.autos.Bottom_Autos.Blue_Auto_Bumper_0_3;
// import frc.robot.autos.Bottom_Autos.Blue_Auto_Bumper_0_3;
import frc.robot.autos.Bottom_Autos.Blue_Auto_Bumper_0_7_Far;
import frc.robot.autos.Bottom_Autos.Blue_Auto_Bumper_0_8_7_Far;
import frc.robot.autos.Bottom_Autos.Blue_Auto_Bumper_0_8_Far;
import frc.robot.autos.Middle_Autos.Blue_Auto_Bumper_0_2_1_Far;
// import frc.robot.autos.Middle_Autos.Blue_Auto_Bumper_0_2_1_Far;
import frc.robot.autos.Middle_Autos.Blue_Auto_Bumper_Middle_0_2_3_1;
import frc.robot.autos.Middle_Autos.Blue_Auto_Bumper_Middle_0_3_2_1;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_0_2;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_0_2_6;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_0_2_Far;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_0_3_2;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_0_3_2_1;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_Bumper_0_2_3;
import frc.robot.autos.Middle_Autos.Blue_Auto_Middle_Bumper_0_3;
import frc.robot.autos.Top_Autos.Blue_Auto_Bumper_0_1;
import frc.robot.autos.Top_Autos.Blue_Auto_Bumper_0_1_4;
import frc.robot.autos.Top_Autos.Blue_Auto_Bumper_0_1_4_5_Far;
import frc.robot.autos.Top_Autos.Blue_Auto_Bumper_0_1_5_Far;
import frc.robot.commands.CANdle_Default;
import frc.robot.commands.CANdle_Intake_Command;
import frc.robot.commands.CANdle_LockOn_Command;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.Elevator_Goto_Angle;
import frc.robot.commands.HangerCommand;
import frc.robot.commands.HangerManualCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommand_Reverse;
import frc.robot.commands.Right_Trigger_Boost_True;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCommand_Top;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand_No_Sensor;
import frc.robot.commands.Transfer_IntakeCommand_Reverse;
import frc.robot.commands.Transfer_IntakeShoot;
import frc.robot.commands.TurretAim;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.CANdle_Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

public class RobotContainer {


  /*Choosers*/
  private final SendableChooser<Double> s_chooser = new SendableChooser<>(); //Shooter velocity chooser
  private final SendableChooser<Integer> a_chooser = new SendableChooser<>(); //Autonomous chooser




  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My driver
  private final CommandXboxController operator = new CommandXboxController(1); // My operator
  private final CommandXboxController testController = new CommandXboxController(2); // My controller buttons for testing
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  /* Subsystems */
  private final Elevator m_Elevator = new Elevator();
  private final Hanger m_Hanger = new Hanger();
  private final Intake m_Intake = new Intake();
  private final Transfer_Intake m_Transfer = new Transfer_Intake();
  private final Turret m_Turret = new Turret();
  private final Shooter m_Shooter = new Shooter();
  private final Auton_Subsystem m_aSub = new Auton_Subsystem();
  private final CANdle_Subsystem m_Candle = new CANdle_Subsystem();
  private final Limelight m_Limelight = new Limelight();


  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  
  

  private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

  private void configureBindings() 
  {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drivetrain.drive.withVelocityX(-driver.getLeftY() * Constants.MaxSpeed * Global_Variables.targetPercentSpeed()) // Drive forward with
                                                                                                                                                      // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * Constants.MaxSpeed * Global_Variables.targetPercentSpeed()) // Drive left with negative X (left)
            .withRotationalRate(drivetrain.swerveRotationRate(operator.x().getAsBoolean(), driver.getRightX()) * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    m_Elevator.setDefaultCommand(new ElevatorCommand(m_Elevator, ()-> -operator.getLeftY())); //-operator.getLeftY()));
    m_Turret.setDefaultCommand(new TurretCommand(m_Turret, ()-> testController.getRightY()));
    m_Hanger.setDefaultCommand(new HangerManualCommand(m_Hanger, ()-> -operator.getRightY()));

    // m_Hanger.setDefaultCommand(new HangerManualCommand_SeperateControl(m_Hanger, ()-> testController.getRightY(), ()-> testController.getLeftY()))

    m_Candle.setDefaultCommand(new CANdle_Default(m_Candle));
    

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    driver.rightTrigger(0.8).whileTrue(new Right_Trigger_Boost_True());
    driver.b().whileTrue(new Transfer_IntakeCommand_No_Sensor(m_Transfer));
    testController.x().whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));        

    new Trigger((() -> operator.getLeftTriggerAxis() > 0.80)).whileTrue(new CANdle_Intake_Command(m_Candle));

    operator.x().whileTrue(new CANdle_LockOn_Command(m_Candle));
    driver.b().whileTrue(new CANdle_LockOn_Command(m_Candle));

    /*Aiming to Score and Rev up Shooter: Driver[RightBumper] */
    operator.x().whileTrue(new TurretAim(m_Turret));
    operator.x().whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
    driver.leftBumper().whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
    driver.leftBumper().whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_DEFAULT_POSITION));

    /**Half Court Shot */
    driver.rightBumper().whileTrue(new ShootCommand(m_Shooter, ()-> 2500));
    driver.rightBumper().whileTrue(new Turret_Goto_Angle(m_Turret, 12.5));

    driver.back().whileTrue(new ShootCommand_Top(m_Shooter, ()-> 0.3));
    driver.back().whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_DEFAULT_POSITION+5));

    /*GOTO Default Position and Rev up Shooter: Driver[LeftBumper]*/
    operator.b().whileTrue(new ShootCommand(m_Shooter, ()-> s_chooser.getSelected()));
    operator.b().whileTrue(new Turret_Goto_Angle(m_Turret, Constants.Turret.TURRET_PROTECTED_POSITION));

    /*Transfer and Floor Intake: Operator[LeftTrigger]*/
    operator.leftTrigger(0.8).whileTrue(new IntakeCommand(m_Intake));
    operator.leftTrigger(0.8).whileTrue(new Transfer_IntakeCommand(m_Transfer));

    /*Transfer Intake Poop: Operator[RightBumper]*/
    operator.rightBumper().whileTrue(new Transfer_IntakeCommand_Reverse(m_Transfer));

    /*Floor Intake Reverse: Operator[LeftBumper] */
    operator.leftBumper().whileTrue(new IntakeCommand_Reverse(m_Intake));

    /*Elevator Setpositions: Operator[A, B, X, Y]*/
    operator.a().whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.A_Button));//////
    operator.y().whileTrue(new Elevator_Goto_Angle(m_Elevator, Constants.Y_Button));//////

    /* Hangar Command: Operator[BackButton, StartButton]*/
    operator.back().whileTrue(new HangerCommand(m_Hanger, true));
    operator.start().whileTrue(new HangerCommand(m_Hanger, false));

    /*Shoot: Operator[RightTrigger]*/
    operator.rightTrigger(0.8).whileTrue(new Transfer_IntakeShoot(m_Transfer));
  }

  public RobotContainer() {
    configureBindings();
    stuffToPrint();
    /*Not Ready to do autonomous */
    // printAutons();
  }

  private void stuffToPrint(){
    SmartDashboard.putData("Shot Speed", s_chooser);

    s_chooser.addOption("1000 rpm", 1000.0);
    s_chooser.addOption("2000 rpm",  2000.0);
    s_chooser.addOption("3000 rpm",  3000.0);
    s_chooser.addOption("3500 rpm",  3500.0);
    s_chooser.setDefaultOption("4000 rpm",  4000.0);

    SmartDashboard.putData("Turret Angle Offset", Global_Variables.tOffset_chooser);

    Global_Variables.tOffset_chooser.addOption("-3.0 degrees", -3.0);
    Global_Variables.tOffset_chooser.addOption("-2.75 degrees", -2.75);
    Global_Variables.tOffset_chooser.addOption("-2.5 degrees", -2.5);
    Global_Variables.tOffset_chooser.addOption("-2.25 degrees", -2.25);
    Global_Variables.tOffset_chooser.addOption("-2.0 degrees", -2.0);
    Global_Variables.tOffset_chooser.addOption("-1.75 degrees", -1.75);
    Global_Variables.tOffset_chooser.addOption("-1.5 degrees", -1.5);
    Global_Variables.tOffset_chooser.addOption("-1.25 degrees", -1.25);
    Global_Variables.tOffset_chooser.addOption("-1.0 degrees", -1.0);
    Global_Variables.tOffset_chooser.addOption("-0.75 degrees", -0.75);
    Global_Variables.tOffset_chooser.addOption("-0.5 degrees", -0.5);
    Global_Variables.tOffset_chooser.setDefaultOption("0.0 degrees",  0.0);
    Global_Variables.tOffset_chooser.addOption("1.0 degrees", 1.0);
    Global_Variables.tOffset_chooser.addOption("1.5 degrees", 1.5);
    Global_Variables.tOffset_chooser.addOption("1.75 degrees", 1.75);
    Global_Variables.tOffset_chooser.addOption("2.0 degrees", 2.0);
    Global_Variables.tOffset_chooser.addOption("2.5 degrees", 2.5);
  }
  private void printAutons(){
    SmartDashboard.putData("Auton", a_chooser);

    a_chooser.setDefaultOption("Leave Zone", 1);
    a_chooser.addOption("0, 2 Bumper Auto", 9);
    a_chooser.addOption("0, 3 Bumper Auto", 10);
    a_chooser.addOption("0, 1 Bumper Auto", 11);
    a_chooser.addOption("0, 8 Bumper Auto", 12);
    a_chooser.addOption("0, 1, 4 Bumper Auto", 14);
    a_chooser.addOption("Test Auto", 15);
    a_chooser.addOption("0, 2 Bumper Far Auto", 16);
    a_chooser.addOption("Shoot Only Auto", 17);
    a_chooser.addOption("0, 7 Bumper Far Auto", 18);
    a_chooser.addOption("0, 1, 5 Bumper Far Auto", 19);
    a_chooser.addOption("0, 7, 8 Bumper Far Auto", 20);
    a_chooser.addOption("0, 1, 4, 5 Bumper Far Auto", 21);
    a_chooser.addOption("0, 2, 1 Bumper Far Auto", 22);
    a_chooser.addOption("0, 2, 3 Middle Bumper Auto", 23);
    a_chooser.addOption("0, 3 Middle Bumper Auto", 24);
    a_chooser.addOption("0, 2, 6 Middle Bumper Auto", 27);
    a_chooser.addOption("0, 3, 2, 1 Middle Bumper Auto", 28);
    a_chooser.addOption("0, 2, 3, 1 Middle Bumper Auto", 29);
}


  public Command getAutonomousCommand() {
    Command autonomous = null;

    switch (a_chooser.getSelected()) 
    {
        case 1: autonomous = new Blue_Leave_Zone(drivetrain);
        case 9: autonomous = new Blue_Auto_Middle_0_2(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 10: autonomous = new Blue_Auto_Bumper_0_3(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 11: autonomous = new Blue_Auto_Bumper_0_1(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 12: autonomous = new Blue_Auto_Bumper_0_8_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 14: autonomous = new Blue_Auto_Bumper_0_1_4(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 15: autonomous = new TestAuton(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake); //TestAuton
        case 16: autonomous = new Blue_Auto_Middle_0_2_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 17: autonomous = new Shoot_Only_Auto(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 18: autonomous = new Blue_Auto_Bumper_0_7_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 19: autonomous = new Blue_Auto_Bumper_0_1_5_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 20: autonomous = new Blue_Auto_Bumper_0_8_7_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 21: autonomous = new Blue_Auto_Bumper_0_1_4_5_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 22: autonomous = new Blue_Auto_Bumper_0_2_1_Far(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 23: autonomous = new Blue_Auto_Middle_Bumper_0_2_3(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 24: autonomous = new Blue_Auto_Middle_Bumper_0_3(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 25: autonomous = new Blue_Auto_Middle_0_3_2_1(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 26: autonomous = new Blue_Auto_Middle_0_3_2(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 27: autonomous = new Blue_Auto_Middle_0_2_6(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 28: autonomous = new Blue_Auto_Bumper_Middle_0_3_2_1(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
        case 29: autonomous = new Blue_Auto_Bumper_Middle_0_2_3_1(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);

        default: autonomous = new Shoot_Only_Auto(drivetrain, m_Turret, m_Shooter, m_aSub, m_Transfer, m_Intake);
    }

    if(autonomous == null){
      return Commands.print("Autonomous selected is null");
    }
    else{
      return autonomous;
    }

  }
}
