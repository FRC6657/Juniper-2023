package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final MecanumDrivePoseEstimator mPoseEstimator;

  private final MecanumSimulation mSimulation;
  
  private Field2d mField;
  private FieldObject2d mTraj;
  private List<Pose2d> mTrajPoints;

  public Drivetrain() {

    mPigeon = new WPI_Pigeon2(Constants.CAN.kPigeon);

    mPigeon.reset();

    mFrontLeft = new WPI_TalonFX(Constants.CAN.kFrontLeft);
    mFrontRight = new WPI_TalonFX(Constants.CAN.kFrontRight);
    mBackRight = new WPI_TalonFX(Constants.CAN.kBackRight);
    mBackLeft = new WPI_TalonFX(Constants.CAN.kBackLeft);

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
    
    mFeedForward = new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV, Constants.DriveConstants.kA);
    
    mPoseEstimator = new MecanumDrivePoseEstimator(
      mKinematics, 
      new Rotation2d(Units.degreesToRadians(-90)),
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

    mField = new Field2d();
    mTraj = mField.getObject("waypoints");
    mTrajPoints = new ArrayList<Pose2d>();
    mTraj.setPoses(mTrajPoints);

    SmartDashboard.putData(mField);

  }

  @Override
  public void periodic() {

    updateOdometry();

    mField.setRobotPose(getPose());
    // Logger.getInstance().recordOutput("Robot Position", getPose());
    // Logger.getInstance().recordOutput("Front Left M/s", getCurrentState().frontLeftMetersPerSecond);

  }

  public void resetGyro() {

    mPigeon.reset();

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

  public double[] getMotorSets() {
    return new double[]{
      mFrontLeft.get(),
      mFrontRight.get(),
      mBackLeft.get(),
      mBackRight.get()
    };
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

    mFrontLeft.setVoltage(MathUtil.clamp(frontLeftFeedForward + frontLeftOutput, -12, 12));
    mFrontRight.setVoltage(MathUtil.clamp(frontRightFeedForward + frontRightOutput, -12, 12));
    mBackLeft.setVoltage(MathUtil.clamp(backLeftFeedForward + backLeftOutput, -12, 12));
    mBackRight.setVoltage(MathUtil.clamp(backRightFeedForward + backRightOutput, -12, 12));

  }

  public Pose2d getPose() {

    return mPoseEstimator.getEstimatedPosition();

  }

  public void resetPose(Pose2d position) {

    mPoseEstimator.resetPosition(mPigeon.getRotation2d(), getCurrentDistances(), position);

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

  public void updateOdometry() {
    mPoseEstimator.update(mPigeon.getRotation2d(), getCurrentDistances());
  }
 
  @Override
  public void simulationPeriodic() {
      mSimulation.update();
  }

  public MecanumDriveKinematics getKinematics() {
    return mKinematics;
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetPose(traj.getInitialHolonomicPose());
          }
        }),
        new PPMecanumControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            this.mKinematics, // MecanumDriveKinematics
            new PIDController(1.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(1.5, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            3.0, // Max wheel velocity meters per second
            this::setSpeeds, // MecanumDriveWheelSpeeds consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
}


}

 