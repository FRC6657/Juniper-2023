package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flipper extends SubsystemBase{
    
    private final WPI_TalonSRX mFlipper; 

    public Flipper() {
        mFlipper = new WPI_TalonSRX(8);
    
    }

    public void run(double speed) {
        mFlipper.set(speed);
    }


    public void stop() {
        mFlipper.set(0);
    }
    
}
