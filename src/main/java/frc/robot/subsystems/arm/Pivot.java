package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;
    private final DoubleSolenoid mSolenoid;
    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    public double mTargetAngle, mCurrentAngle;


    public Pivot() {

        mPivot = new WPI_TalonFX(Constants.CAN.kPivot);
        mSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 8, 9);
        mEncoder = new DutyCycleEncoder(9);
        mPID = new PIDController(8 / 48.2, 0, 0);

        Timer.delay(1);
        mEncoder.setPositionOffset(0/1024);

        mPivot.setSelectedSensorPosition(degreeToFalcon(mEncoder.getAbsolutePosition()) - mEncoder.getPositionOffset());
        mPID.setTolerance(1, 5);

        configureMotor();
    }


    public void configureMotor() {

    }

    public void runPivot() {

       double mPIDEffort = mPID.calculate(mCurrentAngle, mTargetAngle);

       mPivot.setVoltage(mPIDEffort);

    }

    public void ratchetEnable() {
        mSolenoid.set(Value.kForward);
    }

    public void ratchetDisable() {
        mSolenoid.set(Value.kReverse);
    }

    public void forward() {
        mPivot.set(-0.5);
    }

    public void backward() {
        mPivot.set(0.5);
    }

    public void stop() {
        mPivot.set(0);
    }

    public double getAngle() {
        return mPivot.getSelectedSensorPosition() * 1/2048d * 1/100 * 24/60d * 360;
    }

    public double getThroughBoreAngle () {
        return (mEncoder.getAbsolutePosition() / 1024) * 360;
    }

    public void resetFalcon() {
        mPivot.setSelectedSensorPosition(0);
    }

    public double degreeToFalcon(double deg) {
        return deg * 1/(1/2048d * 1/100 * 24/60d * 360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Degrees", getAngle());
        SmartDashboard.putNumber("TBE raw", mEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("TBE corrected", mEncoder.getAbsolutePosition() - mEncoder.getPositionOffset());

    }
}
