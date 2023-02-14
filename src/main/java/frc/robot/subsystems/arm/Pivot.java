package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;
    private final DoubleSolenoid mSolenoid;
    private final Encoder mEncoder;
    
    public Pivot() {

        mPivot = new WPI_TalonFX(Constants.CAN.kPivot);
        mSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.CTREPCM, 0, 1);
        mEncoder = new Encoder(0, 1, false, EncodingType.k4X);

        mEncoder.reset();
        ratchetEnable();

        mEncoder.setDistancePerPulse(0);

    }
    public void ratchetEnable() {
        mSolenoid.set(Value.kForward);
    }

    public void ratchetDisable() {
        mSolenoid.set(Value.kReverse);
    }


    public void forward() {
        mPivot.set(0.5);
    }

    public void backward() {
        mPivot.set(-0.5);
    }

    public void stop() {
        mPivot.set(0);
    }
}
