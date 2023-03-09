package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    private final WPI_TalonFX mArm;
    
    boolean PIDEnable;

    public Arm() {

        mArm = new WPI_TalonFX(Constants.CAN.kArm);
        mArm.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 30, 0));
        mArm.setInverted(true);

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

        Logger.getInstance().recordOutput("Raw Encoder", mArm.getSelectedSensorPosition());
        Logger.getInstance().recordOutput("extention volts", mArm.getMotorOutputVoltage());

    }

}
