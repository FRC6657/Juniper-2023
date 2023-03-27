package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.SETPOINTS;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;

public class StartingConfig extends SequentialCommandGroup {
    
    public StartingConfig(Pivot pivot, Arm arm) {
            pivot.changeSetpoint(SETPOINTS.START.angle);
    }
}
