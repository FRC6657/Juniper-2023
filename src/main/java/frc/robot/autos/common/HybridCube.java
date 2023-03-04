package frc.robot.autos.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Pistons;

public class HybridCube extends SequentialCommandGroup {
    
    public HybridCube(Claw claw, Pivot pivot, Arm arm, Pistons pistons) {
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(pistons::extend),
                new InstantCommand(claw::outtake),
                new WaitCommand(0.5),
                new InstantCommand(claw::stop)
        ));
    }
}
