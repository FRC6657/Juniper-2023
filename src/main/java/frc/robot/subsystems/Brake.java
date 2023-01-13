package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Brake extends SubsystemBase {
    
    private final DoubleSolenoid mRight; 
    //private final DoubleSolenoid mLeft;
    private final PneumaticsControlModule mPCM = new PneumaticsControlModule(0);
    
    
    public Brake() {

        mRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);
        //mLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
        retract();
        
    }

    public void extend() {

        mRight.set(Value.kReverse);
        //mLeft.set(Value.kReverse)

    }

    public void retract() {

        mRight.set(Value.kForward);
        //mLeft.set(Value.kForward);

    }

}
