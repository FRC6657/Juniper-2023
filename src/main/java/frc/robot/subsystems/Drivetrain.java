package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontRight;
  private final WPI_TalonFX mFrontLeft;
  private final WPI_TalonFX mBackRight;
  private final WPI_TalonFX mBackLeft;

  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(Constants.CAN.kPigeon);
  
  private PIDController mPID = new PIDController(0.5, 0, 0);

  Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  //Get real measurements for wheel distance


  MecanumDriveKinematics mKinematics = new MecanumDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);


    public void configureMotors() {

      mFrontRight.configFactoryDefault();
      mFrontLeft.configFactoryDefault();
      mBackRight.configFactoryDefault();
      mBackLeft.configFactoryDefault();

      mFrontRight.setNeutralMode(NeutralMode.Brake);
      mFrontLeft.setNeutralMode(NeutralMode.Brake);
      mBackRight.setNeutralMode(NeutralMode.Brake);
      mBackLeft.setNeutralMode(NeutralMode.Brake);
      
    }
  
    public Drivetrain() {

    mFrontLeft = new WPI_TalonFX(Constants.CAN.kFrontLeft, "rio");
    mFrontRight = new WPI_TalonFX(Constants.CAN.kFrontRight, "rio");
    mBackRight = new WPI_TalonFX(Constants.CAN.kBackRight, "rio");
    mBackLeft = new WPI_TalonFX(Constants.CAN.kBackLeft, "rio");
    
    }
  

  public void Drive(double xPower, double yPower, double zPower) {

    mFrontLeft.set(xPower + yPower + zPower);
    mFrontRight.set(-xPower + yPower + zPower);
    mBackLeft.set(xPower -yPower + zPower);
    mBackRight.set(-xPower -yPower + zPower);
   
  }


}

