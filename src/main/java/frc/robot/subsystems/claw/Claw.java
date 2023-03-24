package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase{

    private final TalonSRX mLeftClaw;
    private final TalonSRX mRightClaw;

    public Claw() {
        
        mLeftClaw = new TalonSRX(Constants.CAN.kLeftClaw);
        mRightClaw = new TalonSRX(Constants.CAN.kRightClaw);

    }

    public void configureMotors() {

        mLeftClaw.configFactoryDefault();
        mRightClaw.configFactoryDefault();

        mLeftClaw.setNeutralMode(NeutralMode.Brake);
        mRightClaw.setNeutralMode(NeutralMode.Brake);
        
        mLeftClaw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
        mRightClaw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
        
        mRightClaw.setInverted(InvertType.InvertMotorOutput);

    }

    public void intake() {
        mLeftClaw.set(ControlMode.PercentOutput, Constants.IntakeConstants.STATE.INTAKE.speed);
        mRightClaw.set(ControlMode.PercentOutput, Constants.IntakeConstants.STATE.INTAKE.speed);
    }

    public void outtake() {
        mLeftClaw.set(ControlMode.PercentOutput, Constants.IntakeConstants.STATE.OUTTAKE.speed);
        mRightClaw.set(ControlMode.PercentOutput, Constants.IntakeConstants.STATE.OUTTAKE.speed);
    }

    public void idle() {
        mLeftClaw.set(ControlMode.PercentOutput,Constants.IntakeConstants.STATE.IDLE.speed);
        mRightClaw.set(ControlMode.PercentOutput, Constants.IntakeConstants.STATE.IDLE.speed);
    }

    public void stop() {
        mLeftClaw.set(ControlMode.PercentOutput, 0);
        mRightClaw.set(ControlMode.PercentOutput, 0);
    }

    
}

