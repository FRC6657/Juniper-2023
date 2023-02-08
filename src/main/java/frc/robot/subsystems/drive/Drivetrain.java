package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonFX mFrontRight;
  private final WPI_TalonFX mFrontLeft;
  private final WPI_TalonFX mBackRight;
  private final WPI_TalonFX mBackLeft;

  private final WPI_Pigeon2 mPigeon;
  
  private final Field2d mField = new Field2d();

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
  private final MecanumDrivePoseEstimator mPoseEstimator;

  private final MecanumSimulation mSimulation;


  public Drivetrain() {

    mPigeon = new WPI_Pigeon2(Constants.CAN.kPigeon);

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

    if(RobotBase.isReal()){
      mFrontRight.setInverted(true);
      mBackRight.setInverted(true);
    }
    
    mFrontRight.setInverted(true);
    mBackRight.setInverted(true);
    
    mFrontLeftPIDController = new PIDController(0.4, 0, 0);
    mFrontRightPIDController = new PIDController(0.4, 0, 0);
    mBackLeftPIDController = new PIDController(0.4, 0, 0);
    mBackRightPIDController = new PIDController(0.4, 0, 0);

    mFrontLeftLocation = new Translation2d(0.291841, 0.258571);
    mFrontRightLocation = new Translation2d(0.291841, -0.258571); 
    mBackLeftLocation = new Translation2d(-0.291841, 0.258571); 
    mBackRightLocation = new Translation2d(-0.291841, -0.258571);

    mKinematics = new MecanumDriveKinematics(
      mFrontLeftLocation, 
      mFrontRightLocation, 
      mBackLeftLocation, 
      mBackRightLocation);
    
    mFeedForward = new SimpleMotorFeedforward(0.13305, 2.2876, 0.31596);
    
    mPoseEstimator = new MecanumDrivePoseEstimator(
      mKinematics, 
      mPigeon.getRotation2d(),
      getCurrentDistances(), 
      new Pose2d()
  );

  mSimulation = new MecanumSimulation(
      new TalonFXSimCollection[]{
        mFrontLeft.getSimCollection(),
        mFrontRight.getSimCollection(),
        mBackLeft.getSimCollection(),
        mBackRight.getSimCollection()
      },
      mPigeon.getSimCollection(),
      Constants.DriveConstants.kDrivetrainCharacterization,
      mKinematics,
      DCMotor.getFalcon500(1), 
      KitbotGearing.k10p71.value, 
      this::getCurrentState,
      this::getMotorSets
    );

  }

  @Override
  public void periodic() {

    mField.setRobotPose(getPose());
    updateOdometry();

  }

  @Override
  public void simulationPeriodic() {

    mSimulation.update();

  }

  public void resetGyro() {

    mPigeon.reset();

  }

  //For Sim
  public double[] getMotorSets() {
    return new double[] {
      mFrontLeft.get(),
      mFrontRight.get(),
      mBackLeft.get(),
      mBackRight.get()
    };
  } 

  // current state of dt velocity
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      mFrontLeft.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
      mFrontRight.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
      mBackLeft.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
      mBackRight.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10);
  }

  //distances measured in meters
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
      mFrontLeft.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
      mFrontRight.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
      mBackLeft.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters, 
      mBackRight.getSelectedSensorPosition() * Constants.DriveConstants.kFalconToMeters);
  }

  // what we want
  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {

    final double frontLeftFeedForward = mFeedForward.calculate(speeds.frontLeftMetersPerSecond);
    final double frontRightFeedForward = mFeedForward.calculate(speeds.frontRightMetersPerSecond);
    final double backLeftFeedForward = mFeedForward.calculate(speeds.rearLeftMetersPerSecond);
    final double backRightFeedForward = mFeedForward.calculate(speeds.rearRightMetersPerSecond);
    
    final double frontLeftOutput =
        mFrontLeftPIDController.calculate(
          mFrontLeft.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
          speeds.frontLeftMetersPerSecond);
    
    final double frontRightOutput =
          mFrontRightPIDController.calculate(
            mFrontRight.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
            speeds.frontRightMetersPerSecond);

    final double backRightOutput =
            mBackRightPIDController.calculate(
              mBackRight.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
              speeds.rearRightMetersPerSecond);
    
    final double backLeftOutput =
              mBackLeftPIDController.calculate(
                mBackLeft.getSelectedSensorVelocity() * Constants.DriveConstants.kFalconToMeters * 10, 
                speeds.rearLeftMetersPerSecond);

    mFrontLeft.setVoltage(frontLeftFeedForward + frontLeftOutput);
    mFrontRight.setVoltage(frontRightFeedForward + frontRightOutput);
    mBackLeft.setVoltage(backLeftFeedForward + backLeftOutput);
    mBackRight.setVoltage(backRightFeedForward + backRightOutput);

    
    //velocity
    Logger.getInstance().recordOutput("back right speeds", speeds.rearRightMetersPerSecond);
    Logger.getInstance().recordOutput("back left speeds", speeds.rearLeftMetersPerSecond);
    Logger.getInstance().recordOutput("front right speeds", speeds.frontRightMetersPerSecond);
    Logger.getInstance().recordOutput("front left speeds", speeds.frontLeftMetersPerSecond);

    //setpoint
    Logger.getInstance().recordOutput("fR set velocity", getCurrentState().frontRightMetersPerSecond);
    Logger.getInstance().recordOutput("fL set velocity", getCurrentState().frontLeftMetersPerSecond);
    Logger.getInstance().recordOutput("bR set velocity", getCurrentState().rearRightMetersPerSecond);
    Logger.getInstance().recordOutput("bL set velocity", getCurrentState().rearLeftMetersPerSecond);

  }

  public Pose2d getPose() {

    return mPoseEstimator.getEstimatedPosition();

  }

  public void resetPose(Pose2d position) {

    resetGyro();
    
    mFrontRight.setSelectedSensorPosition(0);
    mFrontLeft.setSelectedSensorPosition(0);
    mBackRight.setSelectedSensorPosition(0);
    mBackLeft.setSelectedSensorPosition(0);

    mPoseEstimator.resetPosition(position.getRotation(), getCurrentDistances(), position);

  }
  
  public void stop() {
    
    mFrontRight.set(0);
    mFrontLeft.set(0);
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

  }

  public void easyDrive(double xPower, double yPower, double zPower) {

    mFrontLeft.set(0.7 * (xPower + yPower + zPower));
    mFrontRight.set(0.7 * (-xPower + yPower + zPower));
    mBackLeft.set(0.7 * (xPower -yPower + zPower));
    mBackRight.set(0.7 * (-xPower -yPower + zPower));

  }

  public void updateOdometry() {
    mPoseEstimator.update(mPigeon.getRotation2d(), getCurrentDistances());
  }
 
}

