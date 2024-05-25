package frc.robot.autos;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Blue_Leave_Zone extends SequentialCommandGroup {
    public Blue_Leave_Zone(CommandSwerveDrivetrain swerve){
        addRequirements(swerve);

        
        addCommands(
            new InstantCommand(() -> swerve.seedFieldRelative()),
            new PathPlannerAuto("Blue_Leave_Zone")
        );
    }
}
