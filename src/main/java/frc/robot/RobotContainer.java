package frc.robot;

import frc.robot.Constants.IntakeConstants.STATE;
import frc.robot.autos.StartingConfig;
import frc.robot.autos.BlueAlliance.BlueMidTaxiBump;
import frc.robot.autos.BlueAlliance.BlueMid;
import frc.robot.autos.BlueAlliance.BlueMidTaxi;
import frc.robot.autos.BlueAlliance.BlueMidTaxiGrab;
import frc.robot.autos.BlueAlliance.BlueTaxi;
import frc.robot.autos.RedAlliance.RedMidTaxiGrab;
import frc.robot.autos.RedAlliance.RedMidTaxi;
import frc.robot.autos.RedAlliance.RedMidTaxiBump;
import frc.robot.autos.RedAlliance.RedMid;
import frc.robot.autos.RedAlliance.RedTaxi;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.custom.controls.deadbander;

public class RobotContainer {

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  private static final Field2d mField = new Field2d();

  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Claw claw = new Claw();
  private final Pivot pivot = new Pivot();

  private SendableChooser<SequentialCommandGroup[]> mAutoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
  }

 
  private void configureBindings() {

    CommandScheduler.getInstance().setDefaultCommand(drivetrain, 
      new DriverControl(drivetrain, 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightY(), 0.1) * (mDriver.leftTrigger().getAsBoolean() ? Constants.DriveConstants.kTurboSpeed : Constants.DriveConstants.kNormalSpeed), 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getRightX(), 0.1) * (mDriver.leftTrigger().getAsBoolean() ? Constants.DriveConstants.kTurboSpeed : Constants.DriveConstants.kNormalSpeed), 
        ()-> deadbander.applyLinearScaledDeadband(-mDriver.getLeftX(), 0.1) * (Constants.DriveConstants.kNormalRotSpeed), 
        true));

      mDriver.y().onTrue(
        claw.changeState(STATE.EXTEND)
      );

      mDriver.x().onTrue(
        claw.changeState(STATE.RETRACT)
      );

      mDriver.b().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain
        )
      );

      mDriver.a().whileTrue(
        new InstantCommand(
          pivot::zeroEncoder)
      );

      mDriver.rightBumper().whileTrue(
        claw.changeState(STATE.STOP)
      );

      mOperator.leftBumper().whileTrue(
        claw.changeState(STATE.GRAB)
      ).whileFalse(
        claw.changeState(STATE.CLAMP)
      );

      mOperator.rightBumper().whileTrue(
        claw.changeState(STATE.OUTTAKE)
      ).whileFalse(
        claw.changeState(STATE.IDLE)
      );

      mOperator.leftTrigger().whileTrue(
        claw.changeState(STATE.EXTEND)
      );

      mOperator.rightTrigger().whileTrue(
        claw.changeState(STATE.OUTTAKE)
      );

      mOperator.povRight().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.START.angle))
      );

      mOperator.povDown().whileTrue(
        new InstantCommand(
          pivot::zeroEncoder
        )
      );

      pivot.setDefaultCommand(
        new RunCommand(
          () -> pivot.trimTargetAngle(-mOperator.getLeftY() * 10),
           pivot
        )
      );


      arm.setDefaultCommand(
        new RunCommand(
          () -> arm.set(deadbander.applyLinearScaledDeadband(-mOperator.getRightY(), 0.1)),
          arm
        )
      );

      mOperator.x().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.INTAKE.angle)
        )
      );

      mOperator.y().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.ZERO.angle)
        )
      );

      mOperator.b().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.CARRY.angle)
        )
      );

      mOperator.a().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.MID.angle)
        )
      );

      configureAutoChooser();

      }
      
      public void configureAutoChooser() {

        mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null, null});

        mAutoChooser.addOption("Starting Config", new SequentialCommandGroup[] {
          new StartingConfig(pivot, arm),
          new StartingConfig(pivot, arm)
        });

        mAutoChooser.addOption("Taxi", new SequentialCommandGroup[] {
          new BlueTaxi(drivetrain),
          new RedTaxi(drivetrain)
        });

        mAutoChooser.addOption("CubeScoreTaxi", new SequentialCommandGroup[] {
          new BlueMidTaxi(drivetrain, pivot, arm, claw),
          new RedMidTaxi(drivetrain, pivot, arm, claw)
        });

        mAutoChooser.addOption("CubeScoreIntakeScore", new SequentialCommandGroup[] {
          new BlueMidTaxiGrab(drivetrain, pivot, arm, claw),
          new RedMidTaxiGrab(drivetrain, pivot, arm, claw)
        });

        mAutoChooser.addOption("Shoot", new SequentialCommandGroup[] {
          new BlueMid(pivot, arm, claw),
          new RedMid(pivot, arm, claw)
        });

        mAutoChooser.addOption("Bump Shoot Taxi ", new SequentialCommandGroup[] {
          new BlueMidTaxiBump(drivetrain, pivot, arm, claw),
          new RedMidTaxiBump(drivetrain, pivot, arm, claw)
        });


        SmartDashboard.putData("Auto Chooser", mAutoChooser);

      }

      public SequentialCommandGroup getAutonomousCommand() {
        
        int alliance = 0;
        if(DriverStation.getAlliance() == Alliance.Blue){
          alliance = 0;
        }else{
          alliance = 1;
        }

         return mAutoChooser.getSelected()[alliance];
      }

      public static Field2d getField() {
        return mField;
      }

      public Command stopAll() {
        return new ParallelCommandGroup(
          new InstantCommand(pivot::stop),
          new InstantCommand(arm::stop),
          new InstantCommand(drivetrain::stop),
          claw.changeState(STATE.STOP)
        );
      };
    }