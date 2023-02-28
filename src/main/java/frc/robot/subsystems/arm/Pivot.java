package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;
    private final DoubleSolenoid mSolenoid;
    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    public double mTargetAngle;
    public double falconOffset;

    public Pivot() {

        mPivot = new WPI_TalonFX(Constants.CAN.kPivot);
        mSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 6, 7);
        mEncoder = new DutyCycleEncoder(9);
        mPID = new PIDController(32 / 48.2, 0, 0);
        Timer.delay(2);
        mPivot.setInverted(InvertType.InvertMotorOutput);
        mPivot.setSelectedSensorPosition(0);
        mEncoder.setPositionOffset(0.675);
        falconOffset = degreeToFalcon(getThroughBoreAngle());
        mPID.setTolerance(1, 5);

        //Starting target angle, will move when enabled
        mTargetAngle = 45;

        ratchetDisable();
        configureMotor();

    }


    public void configureMotor() {

        mPivot.configFactoryDefault();
        mPivot.setNeutralMode(NeutralMode.Brake);

        mPivot.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,30,30,0));

    }

    public void runPivot() {

       double mPIDEffort = mPID.calculate(getAngle(), mTargetAngle);
       mPivot.setVoltage(mPIDEffort);

    }

    public void changeSetpoint(double setpoint) {
        mTargetAngle = setpoint;
    }

    public void addToTargetAngle(double value){
        mTargetAngle += value;
    }

    public void ratchetEnable() {
        mSolenoid.set(Value.kReverse);
    }

    public void ratchetDisable() {
        mSolenoid.set(Value.kForward);
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

    public double falconToDegrees(double val){
        return val * 1/2048d * 1/100 * 16/60d * 360;
    }

    public double getAngle() {
        return falconToDegrees(mPivot.getSelectedSensorPosition() + falconOffset);
    }

    public double getThroughBoreAngle () {
        return ((mEncoder.getAbsolutePosition()) - mEncoder.getPositionOffset()) * 360;
    }

    public void resetFalcon() {
        mPivot.setSelectedSensorPosition(0);
    }

    public double degreeToFalcon(double deg) {
        return deg * (1d/360d) * (60d/16d) * 100 * 2048;
    }

    @Override
    public void periodic() {

        runPivot();

        Logger.getInstance().recordOutput("TBE raw", mEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput("TBE corrected", mEncoder.getAbsolutePosition() - mEncoder.getPositionOffset());
        Logger.getInstance().recordOutput("target angle", mTargetAngle);
        Logger.getInstance().recordOutput("arm degrees", getAngle());
        Logger.getInstance().recordOutput("TBE angle", getThroughBoreAngle());
        Logger.getInstance().recordOutput("Pivot Volts", mPivot.getMotorOutputVoltage());

    }
}
