package frc.robot.autos.BlueAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.common.HybridCube;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Pistons;
import frc.robot.subsystems.drive.Drivetrain;

public class CubeTaxiBlue extends SequentialCommandGroup {
    
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Blue Taxi", new PathConstraints(3, 3));
    
    public CubeTaxiBlue(Drivetrain drivetrain, Pivot pivot, Arm arm, Pistons pistons, Claw claw) {
        addCommands(
            new InstantCommand(pivot::zeroEncoder),          
            new InstantCommand(            
                () -> pivot.changeSetpoint(0)),
            new WaitCommand(3),
            new HybridCube(claw, pivot, arm, pistons),
            drivetrain.followTrajectoryCommand(trajectory, true),
            new InstantCommand(arm::disablePID),
            new InstantCommand(pivot::zeroEncoder)
        );
    }
}
