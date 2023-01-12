package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Brake extends SubsystemBase {
    
    private final DoubleSolenoid mRight; 
    
    public Brake() {

        mRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        retract();

    }

    public void extend() {

        mRight.set(Value.kReverse);

    }

    public void retract() {

        mRight.set(Value.kForward);

    }

}
