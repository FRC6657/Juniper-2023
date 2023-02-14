package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pistons extends SubsystemBase {
    
    private final DoubleSolenoid mRSolenoid = new DoubleSolenoid(11, PneumaticsModuleType.CTREPCM, 6, 7);
    private final DoubleSolenoid mLSolenoid = new DoubleSolenoid(11, PneumaticsModuleType.CTREPCM, 0, 1);


    public Pistons() {

    

        retract();
    }
    
    public void extend() {

        if(mRSolenoid.get() == Value.kReverse && mLSolenoid.get() == Value.kReverse) {
            mLSolenoid.set(Value.kForward);
            mRSolenoid.set(Value.kForward);
        }
    }



    public void retract() {

        mRSolenoid.set(Value.kReverse);
        mLSolenoid.set(Value.kReverse);
    }

    public void toggle() {

        mRSolenoid.toggle();
        mLSolenoid.toggle();
        
    }
}
