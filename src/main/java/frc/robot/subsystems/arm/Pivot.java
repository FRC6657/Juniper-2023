package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;
    
    public Pivot() {

        mPivot = new WPI_TalonFX(Constants.CAN.kPivot);

    }
    public void run(double speed) {
        mPivot.set(speed);
    }

    public void forward() {
        mPivot.set(0.1);
    }

    public void backward() {
        mPivot.set(-0.1);
    }

    public void stop() {
        mPivot.set(0);
    }
}
