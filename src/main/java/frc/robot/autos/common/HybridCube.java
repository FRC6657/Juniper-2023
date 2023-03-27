package frc.robot.autos.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.STATE;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;

public class HybridCube extends SequentialCommandGroup {
    
    public HybridCube(Claw claw, Pivot pivot, Arm arm) {
        addCommands(
            new SequentialCommandGroup(
                claw.changeState(STATE.OUTTAKE),
                new WaitCommand(0.5),
                claw.changeState(STATE.IDLE)
        ));
    }
}
