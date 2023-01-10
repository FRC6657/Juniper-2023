package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontRight;
  private final WPI_TalonFX mFrontLeft;
  private final WPI_TalonFX mBackRight;
  private final WPI_TalonFX mBackLeft;
  private final WPI_TalonFX mTest;


  public Drivetrain() {

    mFrontLeft = new WPI_TalonFX(1, "rio");
    mFrontRight = new WPI_TalonFX(2, "rio");
    mBackRight = new WPI_TalonFX(3, "rio");
    mBackLeft = new WPI_TalonFX(4, "rio");
    mTest = new WPI_TalonFX(5, "rio");
    
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);
  }

  public void Drive(double xPower, double yPower, double zPower) {

    mFrontLeft.set(xPower + yPower + zPower);
    mFrontRight.set(-xPower + yPower + zPower);
    mBackLeft.set(xPower -yPower + zPower);
    mBackRight.set(-xPower -yPower + zPower);
   
  }

  public void testMotor(double speed) {
    mTest.set(speed);
  }

  public void stopMotor() {
    mTest.set(0);
  }

}

