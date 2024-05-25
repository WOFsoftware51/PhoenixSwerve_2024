package frc.robot.autos;

import frc.robot.commands_Auton.Auton_Wait;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(CommandSwerveDrivetrain swerve){
        addRequirements(swerve);

        String pathFile1 = "Test_Path";
        

        addCommands(
            new InstantCommand(() -> swerve.seedFieldRelative()),
            new Auton_Wait(100),
            new PathPlannerAuto("Test_Auto")
        );
    }
}