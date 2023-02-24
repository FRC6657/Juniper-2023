package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brake extends SubsystemBase {
    
    private final DoubleSolenoid mRight; 
    private final DoubleSolenoid mLeft;
    
    public Brake() {

        mRight = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 4, 5);
        mLeft = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 0, 1);
        
    
        retract();
        
    }

    public void retract() {
        mRight.set(Value.kReverse);
        mLeft.set(Value.kReverse);
    }

    public void extend() {
        mRight.set(Value.kForward);
        mLeft.set(Value.kForward);   
     }

}
