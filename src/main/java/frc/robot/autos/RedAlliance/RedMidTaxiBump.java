package frc.robot.autos.RedAlliance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autos.common.HybridCube;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;

public class RedMidTaxiBump extends SequentialCommandGroup {
    
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Red Taxi Bump", new PathConstraints(3, 3));

    public RedMidTaxiBump(Drivetrain drivetrain, Pivot pivot, Arm arm, Claw claw) {
        addCommands(
            new InstantCommand(pivot::zeroEncoder),          
            new InstantCommand(            
                () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.MID.angle)),
            new WaitCommand(3),
            new HybridCube(claw, pivot, arm),
            drivetrain.followTrajectoryCommand(trajectory, true)
        );
    }
}
