package frc.robot.autos.BlueAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drivetrain;

//To taxi and then balance on station as blue alliance
public class BlueTaxi extends SequentialCommandGroup {
    
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Blue Taxi", new PathConstraints(3, 3));

    public BlueTaxi(Drivetrain drivetrain) {
        addCommands(
            drivetrain.followTrajectoryCommand(trajectory, true)
        );
    }

}
