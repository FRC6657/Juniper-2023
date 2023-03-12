package frc.robot.autos.RedAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.autos.common.HybridCube;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Pistons;
import frc.robot.subsystems.drive.Drivetrain;

public class CubeTaxiRed extends SequentialCommandGroup {
    
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Red Taxi", new PathConstraints(3, 3));
    
    public CubeTaxiRed(Drivetrain drivetrain, Pivot pivot, Arm arm, Pistons pistons, Claw claw) {
        addCommands(
            new InstantCommand(pivot::zeroEncoder),
            new InstantCommand(() -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.SINGLE.angle), pivot),
            new WaitCommand(3),
            new HybridCube(claw, pivot, arm, pistons),
            new InstantCommand(
                () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.INTAKE.angle)
            ),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InstantCommand(
                    pistons::extend,
                    pistons
                ),
                new InstantCommand(
                    claw::intake,
                    claw
                )
                ),
            drivetrain.followTrajectoryCommand(trajectory, true)),
            new InstantCommand(
                pistons::retract,
                pistons),
            new InstantCommand(
                claw::stop,
                claw),
            new InstantCommand(pivot::zeroEncoder)
        );
    }
}
