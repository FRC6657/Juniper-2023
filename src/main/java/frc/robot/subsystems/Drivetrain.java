// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  WPI_TalonFX mFrontRight;
  WPI_TalonFX mFrontLeft;
  WPI_TalonFX mBackRight;
  WPI_TalonFX mBackLeft;
  WPI_TalonFX mTest;


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
  

  

  CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
