package frc.robot.autos.Middle_Autos;

import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.commands_Auton.TurretAim_Auton;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer_Intake;
import frc.robot.subsystems.Turret;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Blue_Auto_Middle_0_2_6 extends SequentialCommandGroup {

    public Blue_Auto_Middle_0_2_6(CommandSwerveDrivetrain swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);
        
        addCommands(
            new InstantCommand(() -> swerve.seedFieldRelative()),
            new ParallelCommandGroup(
                aSub.auton_Shooter_Start(shooter),
                aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new PathPlannerAuto("Blue_0_2"),
                new IntakeCommand(intake)
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret)
            ),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                swerve.followTrajectoryCommand("Blue_2_6"),
                new IntakeCommand(intake)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                swerve.followTrajectoryCommand("Blue_6_2"),
                new IntakeCommand(intake)
            ),
            new ParallelCommandGroup(
                new TurretAim_Auton(turret)
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


