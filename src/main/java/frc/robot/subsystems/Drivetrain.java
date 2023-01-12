package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontRight;
  private final WPI_TalonFX mFrontLeft;
  private final WPI_TalonFX mBackRight;
  private final WPI_TalonFX mBackLeft;

  private final WPI_Pigeon2 mPigeon;
  
  private final PIDController mFrontLeftPIDController;
  private final PIDController mFrontRightPIDController;
  private final PIDController mBackLeftPIDController;
  private final PIDController mBackRightPIDController;

  private final Translation2d mFrontLeftLocation;
  private final Translation2d mFrontRightLocation;
  private final Translation2d mBackLeftLocation;
  private final Translation2d mBackRightLocation;

  private final SimpleMotorFeedforward mFeedForward;

  private final MecanumDriveKinematics mKinematics;  
  private final MecanumDriveOdometry mOdometry;

  public Drivetrain() {

    mPigeon = new WPI_Pigeon2(5);

    mPigeon.reset();

    mFrontLeft = new WPI_TalonFX(Constants.CAN.kFrontLeft, "rio");
    mFrontRight = new WPI_TalonFX(Constants.CAN.kFrontRight, "rio");
    mBackRight = new WPI_TalonFX(Constants.CAN.kBackRight, "rio");
    mBackLeft = new WPI_TalonFX(Constants.CAN.kBackLeft, "rio");

    mFrontRight.setNeutralMode(NeutralMode.Brake);
    mFrontLeft.setNeutralMode(NeutralMode.Brake);
    mBackRight.setNeutralMode(NeutralMode.Brake);
    mBackLeft.setNeutralMode(NeutralMode.Brake);

    mFrontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    mFrontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    mBackRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));
    mBackLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 0, 0));

    mFrontRight.setInverted(true);
    mBackRight.setInverted(true);
    
    mFrontLeftPIDController = new PIDController(0, 0, 0);
    mFrontRightPIDController = new PIDController(0, 0, 0);
    mBackLeftPIDController = new PIDController(0, 0, 0);
    mBackRightPIDController = new PIDController(0, 0, 0);

    mFrontLeftLocation = new Translation2d(0.286, 0.28);
    mFrontRightLocation = new Translation2d(0.286, -0.28);
    mBackLeftLocation = new Translation2d(-0.286, 0.28);
    mBackRightLocation = new Translation2d(-0.286, -0.28);


    mKinematics = new MecanumDriveKinematics(
      mFrontLeftLocation, 
      mFrontRightLocation, 
      mBackLeftLocation, 
      mBackRightLocation);

    mOdometry = new MecanumDriveOdometry(mKinematics, mPigeon.getRotation2d(), getCurrentDistances());
    mFeedForward = new SimpleMotorFeedforward(0.13305, 2.2876, 0.31596);

    

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("gyro angle", mPigeon.getAngle());

  }

  public void resetGyro() {

    mPigeon.reset();

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

    SmartDashboard.putNumber("front left", mFrontLeft.get());
    SmartDashboard.putNumber("front right", mFrontRight.get());
    SmartDashboard.putNumber("back left", mBackLeft.get());
    SmartDashboard.putNumber("back right", mBackRight.get());
    
    SmartDashboard.putNumber("back right speeds", speeds.rearRightMetersPerSecond);

  }

  public void easyDrive(double xPower, double yPower, double zPower) {

    mFrontLeft.set(0.7 * (xPower + yPower + zPower));
    mFrontRight.set(0.7 * (-xPower + yPower + zPower));
    mBackLeft.set(0.7 * (xPower -yPower + zPower));
    mBackRight.set(0.7 * (-xPower -yPower + zPower));

  }

  
  public void stop() {

    mBackRight.set(0);
    mBackLeft.set(0);

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var mecanumDriveWheelSpeeds =
        mKinematics.toWheelSpeeds(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, mPigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    setSpeeds(mecanumDriveWheelSpeeds);

    //5 - motor rpm / 10.71 * .4775 / 60

  }

  public void updateOdometry() {
    mOdometry.update(mPigeon.getRotation2d(), getCurrentDistances());
  }
}

