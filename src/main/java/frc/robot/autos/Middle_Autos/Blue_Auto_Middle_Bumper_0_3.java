package frc.robot.autos.Middle_Autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Transfer_IntakeCommand;
import frc.robot.commands.Transfer_IntakeShoot;
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

public class Blue_Auto_Middle_Bumper_0_3 extends SequentialCommandGroup {

    public Blue_Auto_Middle_Bumper_0_3(CommandSwerveDrivetrain swerve, Turret turret, Shooter shooter, Auton_Subsystem aSub, Transfer_Intake transfer, Intake intake){
        
        addRequirements(swerve, turret, shooter, aSub, transfer, intake);

        ////////TODO
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
                new IntakeCommand(intake),
                new PathPlannerAuto("Blue_Middle_Bumper_0_3")
            ),
            new ParallelRaceGroup( 
                new Auton_Wait(50), //Wait at piece 3 for 1 second to ensure we collect the piece
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake)
            ),
            new ParallelRaceGroup(
                new Transfer_IntakeCommand(transfer),
                new IntakeCommand(intake),
                swerve.followTrajectoryCommand("Blue_Middle_Bumper_3_0")
            ),
            aSub.auton_Turret_Start(turret, Constants.Turret.TURRET_DEFAULT_POSITION),
            new ParallelRaceGroup(
                new Auton_Wait(100),
                aSub.auton_Shoot(transfer)
            ),
            new InstantCommand(() -> swerve.seedFieldRelative()),
            aSub.auton_Stop_Shooter(shooter)
        );
    }
}