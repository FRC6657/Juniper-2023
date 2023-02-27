package frc.robot.autos.RedAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drivetrain;

public class TaxiRed extends SequentialCommandGroup{

    //Actually make this 
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("red taxi", new PathConstraints(3, 3));

    public TaxiRed(Drivetrain drivetrain) {
        addCommands(
            drivetrain.followTrajectoryCommand(trajectory, true)
        );
    }
}
