package frc.robot.autos.BlueAlliance;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.STATE;
import frc.robot.autos.common.HybridCube;
import frc.robot.autos.common.IntakeCube;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;

public class BlueMidTaxiGrab extends SequentialCommandGroup {
    
    PathPlannerTrajectory startTrajectory = PathPlanner.loadPath("Blue Taxi", new PathConstraints(3, 3));
    PathPlannerTrajectory secondTrajectory = PathPlanner.loadPath("Blue Intake Cube", new PathConstraints(3, 3));

    public BlueMidTaxiGrab(Drivetrain drivetrain, Pivot pivot, Arm arm, Claw claw) {
        addCommands(
            new InstantCommand(pivot::zeroEncoder),  
            new InstantCommand(drivetrain::resetGyro),        
            new InstantCommand(            
                () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.ZERO.angle)),
            new WaitCommand(1),
            new HybridCube(claw, pivot, arm),
            drivetrain.followTrajectoryCommand(startTrajectory, true),
            new InstantCommand(
                () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.INTAKE.angle)),
            new WaitCommand(1),
            new HybridCube(claw, pivot, arm),
            drivetrain.followTrajectoryCommand(secondTrajectory, false),
            new IntakeCube(claw, pivot, arm),
            new InstantCommand(pivot::zeroEncoder),
            claw.changeState(STATE.IDLE)
        );
    }
}
