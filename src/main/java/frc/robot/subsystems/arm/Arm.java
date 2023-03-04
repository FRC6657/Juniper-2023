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

    public Arm() {

        mArm = new WPI_TalonFX(Constants.CAN.kArm);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
        mArm.setInverted(true);
        mPID = new PIDController(10/4, 0, 0);

        configureMotor();

    }

    public void configureMotor() {
        
        mArm.configFactoryDefault();
        mArm.setNeutralMode(NeutralMode.Brake);

        mArm.configVoltageCompSaturation(10);
        mArm.enableVoltageCompensation(true);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));

    }

    public void changeSetpoint(double setpoint) {

        setpoint = mTargetExtension;

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

        double PIDEffort = mPID.calculate(getCurrentDistance(), MathUtil.clamp(mTargetExtension, 0, 4));
        mArm.set(PIDEffort / 12);

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

    @Override
    public void periodic() {

        Logger.getInstance().recordOutput("Raw Encoder", mArm.getSelectedSensorPosition());
        Logger.getInstance().recordOutput("Arm Meters", getCurrentDistance());
        Logger.getInstance().recordOutput("Extension Setpoint", mTargetExtension);

        runArm();
    }

}
