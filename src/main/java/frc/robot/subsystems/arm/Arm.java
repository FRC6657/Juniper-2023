package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    private final WPI_TalonFX mArm;
    private final PIDController mPID;
    private double mTargetExtension;
    
    boolean PIDEnable;

    public Arm() {

        mArm = new WPI_TalonFX(Constants.CAN.kArm);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
        mArm.setInverted(true);
        mPID = new PIDController(20/(Units.inchesToMeters(37)), 0, 0);

        PIDEnable = false;

        configureMotor();

    }

    public void configureMotor() {
        
        mArm.configFactoryDefault();
        mArm.setNeutralMode(NeutralMode.Brake);

        mArm.configVoltageCompSaturation(10);
        mArm.enableVoltageCompensation(true);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0));

    }

    public void set(double val) {
        mArm.set(val);
    }

    public void changeSetpoint(double setpoint) {

        mTargetExtension = Units.inchesToMeters(setpoint);

    }


    public void addToSetpoint(double value) {
        mTargetExtension += value;
    }


    public double getCurrentDistance() {
        return falconToMeters(mArm.getSelectedSensorPosition());
    }

    public double falconToMeters (double val){
        return val * 1/2048d * 1/16 * Units.inchesToMeters(0.95) * Math.PI;
    }

    public void runArm() {

        if (PIDEnable == true) {

            double PIDEffort = mPID.calculate(getCurrentDistance(), MathUtil.clamp(mTargetExtension, 0, Units.inchesToMeters(35)));
            mArm.set(PIDEffort / 12);

        }

     }

     public void disablePID() {

        this.PIDEnable = false;

     }

    public void extend() {
        mArm.set(0.6);
    }

    public void retract() {
        mArm.set(-0.6);
    }

    public void stop() {
        mArm.set(0);
    }

    public void resetEncoder() {
        mArm.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {

        runArm();

        Logger.getInstance().recordOutput("Raw Encoder", mArm.getSelectedSensorPosition());
        Logger.getInstance().recordOutput("Arm Meters", getCurrentDistance());
        Logger.getInstance().recordOutput("Extension Setpoint", mTargetExtension);
        Logger.getInstance().recordOutput("extention volts", mArm.getMotorOutputVoltage());
    }

}
