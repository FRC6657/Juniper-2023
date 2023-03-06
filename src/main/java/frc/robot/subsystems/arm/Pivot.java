package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
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

    public double mTargetAngle;
    public double falconOffset;
    public double trimVal = 0;

    public Pivot() {

        mPivot = new WPI_TalonFX(Constants.CAN.kPivot);
        mSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 6, 7);
        mEncoder = new DutyCycleEncoder(9);
        mPID = new PIDController(32 / 48.2, 0, 0);
        Timer.delay(2);
        mPivot.setInverted(InvertType.InvertMotorOutput);
        mPivot.setSelectedSensorPosition(0);
        mEncoder.setPositionOffset(0.6789);
        falconOffset = degreeToFalcon(getThroughBoreAngle());
        mPID.setTolerance(1, 5);

        ratchetDisable();
        configureMotor();
        startConfig();

    }

    public void configureMotor() {

        mPivot.configFactoryDefault();
        mPivot.setNeutralMode(NeutralMode.Brake);

        mPivot.configVoltageCompSaturation(10);
        mPivot.enableVoltageCompensation(true);
        mPivot.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
    }

    public void runPivot() {

        if(mPID.atSetpoint() == true) {

            ratchetEnable();
            mPivot.set(0);

        }else{
            ratchetDisable();

            double mPIDEffort = mPID.calculate(
                getAngle(), 
                MathUtil.clamp(mTargetAngle + trimVal, -20, 80));

            mPivot.set(mPIDEffort / 12);
        }

    }

    public void changeSetpoint(double setpoint) {

        if (setpoint < getAngle()) {
            ratchetDisable();
        }

        mTargetAngle = setpoint;

    }

    public void trimTargetAngle(double value){
        trimVal = value;
    }

    public void ratchetEnable() {
        mSolenoid.set(Value.kReverse);
    }

    public void ratchetDisable() {
        mSolenoid.set(Value.kForward);
    }
  
    public void stop() {
        mPivot.set(0);
    }

    public double falconToDegrees(double val){
        return val * 1/2048d * 1/100 * 16/60d * 360;
    }

    public void zeroEncoder() {
        falconOffset = degreeToFalcon(getThroughBoreAngle());
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

    public void startConfig() {
        mTargetAngle = 66;
    }

    @Override
    public void periodic() {

        runPivot();

        SmartDashboard.putNumber("setpoint", mTargetAngle);
        Logger.getInstance().recordOutput("TBE raw", mEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput("TBE corrected", mEncoder.getAbsolutePosition() - mEncoder.getPositionOffset());
        Logger.getInstance().recordOutput("target angle", mTargetAngle);
        Logger.getInstance().recordOutput("arm degrees", getAngle());
        Logger.getInstance().recordOutput("TBE angle", getThroughBoreAngle());
        Logger.getInstance().recordOutput("Pivot Volts", mPivot.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("at setpoint?", mPID.atSetpoint());

    }
}