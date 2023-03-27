package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.STATE;

public class Claw extends SubsystemBase {

    private final TalonSRX mLeftClaw;
    private final TalonSRX mRightClaw;

    private final DoubleSolenoid mLeftPiston = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 8, 9);
    private final DoubleSolenoid mRightPiston = new DoubleSolenoid(Constants.CAN.kPCM, PneumaticsModuleType.REVPH, 12, 13);

    private STATE mCurrentState;

    public Claw() {
        
        mLeftClaw = new TalonSRX(Constants.CAN.kLeftClaw);
        mRightClaw = new TalonSRX(Constants.CAN.kRightClaw);

        mCurrentState = STATE.STARTING;
    }

    public void configureMotors() {

        mLeftClaw.configFactoryDefault();
        mRightClaw.configFactoryDefault();

        mLeftClaw.setNeutralMode(NeutralMode.Brake);
        mRightClaw.setNeutralMode(NeutralMode.Brake);
        
        mLeftClaw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
        mRightClaw.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
        
        mRightClaw.setInverted(true);

    }

    public void setPistons(Value state) {
        mLeftPiston.set(state);
        mRightPiston.set(state);
    }

    public void setMotors(double value) {
        mLeftClaw.set(ControlMode.PercentOutput, value);
        mRightClaw.set(ControlMode.PercentOutput, value);
    }

    private void runClaw() {
      setMotors(mCurrentState.speed);
      setPistons(mCurrentState.value);
  }

    public Command changeState(STATE state){
        return new InstantCommand(() -> mCurrentState = state);
      }
    
      
    @Override
      public void periodic() {
        runClaw();
      }
    }

