package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pistons extends SubsystemBase {
    
    private final DoubleSolenoid mRSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 12, 13);
    private final DoubleSolenoid mLSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 8, 9);


    public Pistons() {
        extend();
    }
    
    public void extend() {
        
            mLSolenoid.set(Value.kForward);
            mRSolenoid.set(Value.kForward);
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
