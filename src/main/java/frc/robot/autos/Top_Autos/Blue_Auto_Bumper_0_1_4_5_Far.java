package frc.robot.autos.Top_Autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands.Turret_Goto_Angle;
import frc.robot.commands_Auton.AutonSwerveAim;
import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.commands_Auton.TurretAim_Auton;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Blue_Auto_Bumper_0_1_4_5_Far extends SequentialCommandGroup {

    public Blue_Auto_Bumper_0_1_4_5_Far(CommandSwerveDrivetrain swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        
        addCommands(
            new InstantCommand(() -> swerve.seedFieldRelativeAngle(60)),
            new ParallelCommandGroup(
                aSub.auton_Shooter_Start(shooter),
                aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Turret_Goto_Angle(turret, Constants.AutonTurretPositions.Top.Position_1),
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                new PathPlannerAuto("Blue_Top_Bumper_0_1")
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret),
                new AutonSwerveAim(swerve, ()-> 0.0, ()-> 0.0)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Turret_Goto_Angle(turret, Constants.AutonTurretPositions.Top.Position_2),
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_Top_Bumper_1_4")
            ),
            new ParallelRaceGroup(
                new Turret_Goto_Angle(turret, Constants.AutonTurretPositions.Top.Position_2),
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_Top_Bumper_4_Shoot")
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret),
                new AutonSwerveAim(swerve, ()-> 0.0, ()-> 0.0)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Turret_Goto_Angle(turret, Constants.AutonTurretPositions.Top.Position_2),
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_4_Far_5")
            ), //
            new ParallelRaceGroup(  
                new Turret_Goto_Angle(turret, Constants.AutonTurretPositions.Top.Position_2),
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_Top_Bumper_5_Shoot")
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret),
                new AutonSwerveAim(swerve, ()-> 0.0, ()-> 0.0)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new InstantCommand(() -> swerve.seedFieldRelative()),
            aSub.auton_Stop_Shooter(shooter)
        );    
    }
}

