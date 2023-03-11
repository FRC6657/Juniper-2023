package frc.robot;

import frc.robot.autos.StartingConfig;
import frc.robot.autos.BlueAlliance.BlueShoot;
import frc.robot.autos.BlueAlliance.CubeTaxiBlue;
import frc.robot.autos.BlueAlliance.CubeTaxiCubeBlue;
import frc.robot.autos.BlueAlliance.TaxiBlue;
import frc.robot.autos.RedAlliance.CubeTaxiCubeRed;
import frc.robot.autos.RedAlliance.CubeTaxiRed;
import frc.robot.autos.RedAlliance.RedCubeTaxiBump;
import frc.robot.autos.RedAlliance.RedShoot;
import frc.robot.autos.RedAlliance.TaxiRed;
import frc.robot.commands.DriverControl;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Pivot;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.Pistons;
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
  private final Pistons pistons = new Pistons();
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
        new InstantCommand(
          pistons::extend,
          pistons
        )
      );

      mDriver.x().onTrue(
        new InstantCommand(
          pistons::retract,
          pistons
        )
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

      mOperator.leftBumper().whileTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            pistons::extend,
            pistons
          ),
          new InstantCommand(
            claw::intake,
            claw
          )
        )
      ).whileFalse(
        new SequentialCommandGroup(
          new InstantCommand(
            pistons::retract,
            pistons
          ),
          new InstantCommand(
            claw::stop,
            claw
          )
        )
      );

      mOperator.rightBumper().whileTrue(
        new SequentialCommandGroup(
          new InstantCommand(
            pistons::extend,
            pistons
          ),
          new InstantCommand(
            claw::outtake,
            claw
          )
        )
      ).whileFalse(
        new InstantCommand(
          claw::stop,
          claw
        )
      );

      mOperator.leftTrigger().whileTrue(
        new InstantCommand(
          claw::intake
        )
      );

      mOperator.rightTrigger().whileTrue(
        new InstantCommand(
          claw::outtake
        )
      );

      mOperator.povRight().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.START.angle))
      );

      mOperator.start().whileTrue(
        new InstantCommand(
          pivot::ratchetEnable, 
          pivot)
      );

      mOperator.back().whileTrue(
        new InstantCommand(
          pivot::ratchetDisable, 
          pivot)
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
          () -> pivot.changeSetpoint(Constants.PivotConstants.SETPOINTS.SINGLE.angle)
        )
      );

      configureAutoChooser();

      }
      
      public void configureAutoChooser() {

        mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null, null});

        mAutoChooser.addOption("Starting Config", new SequentialCommandGroup[] {
          new StartingConfig(pivot, arm, pistons),
          new StartingConfig(pivot, arm, pistons)
        });

        mAutoChooser.addOption("Taxi", new SequentialCommandGroup[] {
          new TaxiBlue(drivetrain),
          new TaxiRed(drivetrain)
        });

        mAutoChooser.addOption("CubeScoreTaxi", new SequentialCommandGroup[] {
          new CubeTaxiBlue(drivetrain, pivot, arm, pistons, claw),
          new CubeTaxiRed(drivetrain, pivot, arm, pistons, claw)
        });

        mAutoChooser.addOption("CubeScoreIntakeScore", new SequentialCommandGroup[] {
          new CubeTaxiCubeBlue(drivetrain, pivot, arm, pistons, claw),
          new CubeTaxiCubeRed(drivetrain, pivot, arm, pistons, claw)
        });

        mAutoChooser.addOption("Shoot", new SequentialCommandGroup[] {
          new BlueShoot(pivot, arm, pistons, claw),
          new RedShoot(pivot, arm, pistons, claw)
        });

        mAutoChooser.addOption("Bump Shoot Taxi ", new SequentialCommandGroup[] {
          null,
          new RedCubeTaxiBump(drivetrain, pivot, arm, pistons, claw)
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
          new InstantCommand(claw::stop)
        );
      };
    }