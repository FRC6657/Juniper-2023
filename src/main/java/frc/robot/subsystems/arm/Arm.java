package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    private final WPI_TalonFX mArm;

    public Arm() {

        mArm = new WPI_TalonFX(Constants.CAN.kArm);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));

    }

    public void extend() {
        mArm.set(0.4);
    }

    public void retract() {
        mArm.set(-0.4);
    }

    public void stop() {
        mArm.set(0);
    }

}
