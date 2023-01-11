package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontRight;
  private final WPI_TalonFX mFrontLeft;
  private final WPI_TalonFX mBackRight;
  private final WPI_TalonFX mBackLeft;

  private final WPI_Pigeon2 mPigeon = new WPI_Pigeon2(5);
  
  private final PIDController mFrontLeftPIDController = new PIDController(0.5, 0, 0);
  private final PIDController mFrontRightPIDController = new PIDController(0.5, 0, 0);
  private final PIDController mBackLeftPIDController = new PIDController(0.5, 0, 0);
  private final PIDController mBackRightPIDController = new PIDController(0.5, 0, 0);

  private final Translation2d mFrontLeftLocation = new Translation2d(0.286, 0.28);
  private final Translation2d mFrontRightLocation = new Translation2d(0.286, -0.28);
  private final Translation2d mBackLeftLocation = new Translation2d(-0.286, 0.28);
  private final Translation2d mBackRightLocation = new Translation2d(-0.286, 0.28);

  private final SimpleMotorFeedforward mFeedForward = new SimpleMotorFeedforward(0.359, 1.2, 0.14);

  private final MecanumDriveKinematics mKinematics =
      new MecanumDriveKinematics(mFrontLeftLocation, mFrontRightLocation, mBackLeftLocation, mBackRightLocation);

  private final MecanumDriveOdometry mOdometry =
        new MecanumDriveOdometry(mKinematics, mPigeon.getRotation2d(), getCurrentDistances());

  public Drivetrain() {

    mPigeon.reset();

    mFrontLeft = new WPI_TalonFX(Constants.CAN.kFrontLeft, "rio");
    mFrontRight = new WPI_TalonFX(Constants.CAN.kFrontRight, "rio");
    mBackRight = new WPI_TalonFX(Constants.CAN.kBackRight, "rio");
    mBackLeft = new WPI_TalonFX(Constants.CAN.kBackLeft, "rio");
    
    
    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);

  }

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
      mFrontRight.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
      mBackLeft.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
      mBackRight.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters);
  }

  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
      mFrontLeft.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
      mFrontRight.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
      mBackLeft.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
      mBackRight.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    final double frontLeftFeedForward = mFeedForward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedForward = mFeedForward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedForward = mFeedForward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedForward = mFeedForward.calculate(speeds.rearRightMetersPerSecond);
    
    final double frontLeftOutput =
        mFrontLeftPIDController.calculate(
          mFrontLeft.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
          speeds.frontLeftMetersPerSecond);
    
    final double frontRightOutput =
          mFrontRightPIDController.calculate(
            mFrontRight.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
            speeds.frontRightMetersPerSecond);

    final double backRightOutput =
            mBackRightPIDController.calculate(
              mBackRight.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
              speeds.rearRightMetersPerSecond);
    
    final double backLeftOutput =
              mBackLeftPIDController.calculate(
                mBackLeft.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
                speeds.rearLeftMetersPerSecond);

    mFrontLeft.setVoltage(frontLeftFeedForward + frontLeftOutput);
    mFrontRight.setVoltage(frontRightFeedForward + frontRightOutput);
    mBackLeft.setVoltage(backLeftFeedForward + backLeftOutput);
    mBackRight.setVoltage(backRightFeedForward + backRightOutput);

  }

  public void easyDrive(double xPower, double yPower, double zPower) {

    mFrontLeft.set(0.7 * (xPower + yPower + zPower));
    mFrontRight.set(0.7 * (-xPower + yPower + zPower));
    mBackLeft.set(0.7 * (xPower -yPower + zPower));
    mBackRight.set(0.7 * (-xPower -yPower + zPower));

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds =
        mKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, mPigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    mecanumDriveWheelSpeeds.desaturate(0.7);
    setSpeeds(mecanumDriveWheelSpeeds);
  }

  public void updateOdometry() {
    mOdometry.update(mPigeon.getRotation2d(), getCurrentDistances());
  }
}

