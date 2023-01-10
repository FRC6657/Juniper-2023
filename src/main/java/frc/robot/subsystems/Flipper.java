package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flipper extends SubsystemBase{
    
    private final Spark mFlipper;

    public Flipper() {
        mFlipper = new Spark(Constants.CAN.kFlipper);
    
    }

    public void run(double speed) {
        mFlipper.set(speed);
    }


    public void stop() {
        mFlipper.set(0);
    }
    
}
