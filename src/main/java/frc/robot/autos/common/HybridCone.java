package frc.robot.autos.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Pistons;

public class HybridCone extends SequentialCommandGroup {
    
    public HybridCone(Pistons pistons, Pivot pivot, Arm arm) {
        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(
                    (() -> pivot.changeSetpoint(-5)),
                    pivot)),
                new WaitCommand(1),
                new InstantCommand(
                    arm::extend,
                    arm),
                new WaitCommand(2),
                new InstantCommand(
                    arm::stop,
                    arm),
                new WaitCommand(1),
                new InstantCommand(pistons::extend),
                new WaitCommand(2),
                new InstantCommand(
                    arm::retract,
                    arm),
                new WaitCommand(2),
                new InstantCommand(
                    arm::retract
                )
        );
    }
}
