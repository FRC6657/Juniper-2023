package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brake extends SubsystemBase {
    
    private final DoubleSolenoid mSolenoid;     
    
    public Brake() {

        mSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.CTREPCM, 3, 4);
        retract();
        
    }

    public void extend() {

        mSolenoid.set(Value.kReverse);

    }

    public void retract() {

        mSolenoid.set(Value.kForward);

    }

}
