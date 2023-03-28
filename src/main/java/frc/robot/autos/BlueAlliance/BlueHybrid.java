package frc.robot.autos.BlueAlliance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.STATE;
import frc.robot.autos.common.HybridCube;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;

public class BlueHybrid extends SequentialCommandGroup {
        
    public BlueHybrid(Pivot pivot, Arm arm, Claw claw) {
        addCommands(
            new InstantCommand(pivot::zeroEncoder),          
            new InstantCommand(            
                () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.HYBRID.angle)),
            new WaitCommand(3),
            new HybridCube(claw, pivot, arm),
            claw.changeState(STATE.IDLE),
                new InstantCommand(            
                    () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.INTAKE.angle)),
                new InstantCommand(pivot::zeroEncoder));
    }
}
