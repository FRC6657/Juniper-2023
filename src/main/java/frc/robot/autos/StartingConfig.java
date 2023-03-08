package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Pistons;

public class StartingConfig extends SequentialCommandGroup {
    
    public StartingConfig(Pivot pivot, Arm arm, Pistons pistons) {
        addCommands(
            new InstantCommand(pivot::startConfig),
            new WaitCommand(2),
            new InstantCommand(
                () -> arm.changeSetpoint(0)
            ),
            new InstantCommand(pistons::extend)
        );

    }
}
