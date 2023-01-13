package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TestAuto extends SequentialCommandGroup{
    
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("rot", new PathConstraints(3, 2));
    
    public TestAuto(Drivetrain drivetrain) {
        addCommands(
            drivetrain.followTrajectoryCommand(trajectory, true)
        );
    }
}
