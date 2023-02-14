package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flipper extends SubsystemBase{
    
    private final Spark mFlipper;

    public Flipper() {
        mFlipper = new Spark(13);
    
    }

    public void run(double speed) {
        mFlipper.set(speed);
    }


    public void stop() {
        mFlipper.set(0);
    }
    
}
