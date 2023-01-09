package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    
    WPI_TalonFX mMotor;

    public Arm() {
        mMotor = new WPI_TalonFX(5, "rio");
    }

    public void run(double speed) {
        mMotor.set(speed);
    }
}

//To Do - two buttons each for set stage of, how to set to certain point 
