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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    
    private final WPI_TalonFX mPivot;
    private final DoubleSolenoid mSolenoid;
    private final DutyCycleEncoder mEncoder;
    private final PIDController mPID;

    public double mTargetAngle = Constants.PivotConstants.SETPOINTS.START.angle;
    public double falconOffset;
    public double trimVal = 0;

    public boolean firstRun = true; 

    public Pivot() {

        mPivot = new WPI_TalonFX(Constants.CAN.kPivot);
        mSolenoid = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 6, 7);
        mEncoder = new DutyCycleEncoder(9);
        mPID = new PIDController(2.5 / 3d, 0, 0);
       
        Timer.delay(2);
        
        mPivot.setInverted(InvertType.InvertMotorOutput);
        mPivot.setSelectedSensorPosition(0);
        mEncoder.setPositionOffset(Constants.PivotConstants.throughboreOffset);
        falconOffset = degreeToFalcon(getThroughBoreAngle());
        mPID.setTolerance(3, 7);

        configureMotor();
        
    }

    public void configureMotor() {

        mPivot.configFactoryDefault();
        mPivot.setNeutralMode(NeutralMode.Brake);

        mPivot.configVoltageCompSaturation(10);
        mPivot.enableVoltageCompensation(true);
        mPivot.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
    }

    public void runPivot() {

        if(atTarget()) {

            mPivot.set(0);
            ratchetEnable();
            firstRun = true;

        }else{
            ratchetDisable();

            if(firstRun) {
                Timer.delay(0.1);
            }

            double mPIDEffort = mPID.calculate(
                getAngle(), 
                MathUtil.clamp(mTargetAngle + trimVal, -20, 80));

            mPivot.set(mPIDEffort / 12);
        
            firstRun = false;
        }
    }

    public boolean atTarget() {

        double tolerance = 2; 

        return (Math.abs(getAngle() - mTargetAngle + trimVal) < tolerance);

    }

    public void changeSetpoint(double setpoint) {
        
        mTargetAngle = setpoint;

    }

    public void trimTargetAngle(double value){
        trimVal = value;
    }


    public void ratchetEnable() {
        mSolenoid.set(Value.kForward);
    }

    public void ratchetDisable() {
        mSolenoid.set(Value.kReverse);
    }
  
    public void stop() {
        mPivot.set(0);
    }

    public double falconToDegrees(double val){
        return val * 1/2048d * 1/100 * 16/60d * 360;
    }

    public void zeroEncoder() {
        mPivot.setSelectedSensorPosition(0);
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
        mTargetAngle = Constants.PivotConstants.SETPOINTS.START.angle;
    }

    @Override
    public void periodic() {

        runPivot();

      
        Logger.getInstance().recordOutput("Pivot/TBE raw", mEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput("Pivot/TBE corrected", mEncoder.getAbsolutePosition() - mEncoder.getPositionOffset());
        Logger.getInstance().recordOutput("Pivot/target angle", mTargetAngle);
        Logger.getInstance().recordOutput("Pivot/arm degrees", getAngle());
        Logger.getInstance().recordOutput("Pivot/TBE angle", getThroughBoreAngle());
        Logger.getInstance().recordOutput("Pivot/Pivot Volts", mPivot.getMotorOutputVoltage());
        Logger.getInstance().recordOutput("Pivot/at setpoint?", atTarget());

    }
}