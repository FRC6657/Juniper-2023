package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pistons extends SubsystemBase {
    
    private final DoubleSolenoid mRSolenoid;
    private final DoubleSolenoid mLSolenoid;

    public Pistons() {

        mLSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.CTREPCM, 6, 7);
        mRSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.CTREPCM, 2, 3);
        
        retract();
    }
    
    public void extend() {

        mRSolenoid.set(Value.kForward);
        mLSolenoid.set(Value.kForward);
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
