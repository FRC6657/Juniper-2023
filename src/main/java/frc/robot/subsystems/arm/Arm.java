package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    private final WPI_VictorSPX mArm;

    public Arm() {

        mArm = new WPI_VictorSPX(8);

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
