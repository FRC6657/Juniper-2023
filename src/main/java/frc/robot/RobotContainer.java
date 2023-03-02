package frc.robot;

import frc.robot.autos.BlueAlliance.TaxiBlue;
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
  private CommandXboxController mTieuTam = new CommandXboxController(2);
  private CommandXboxController mLiam = new CommandXboxController(3);


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
        ()-> deadbander.applyLinearScaledDeadband(-mTieuTam.getRightY(), 0.1) *1.7, 
        ()-> deadbander.applyLinearScaledDeadband(-mTieuTam.getRightX(), 0.1) * 1.7, 
        ()-> deadbander.applyLinearScaledDeadband(-mTieuTam.getLeftX(), 0.1) * 3, 
        true));
    
      // Resets the gyro, specifically for when field relative driving is on
      mDriver.x().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain)
      );

      mOperator.a().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(-10)
        )
      );

      mOperator.b().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(0)
        )
      );

      mOperator.y().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(30)
        )
      );

      mOperator.x().whileTrue(
        new ParallelCommandGroup(
          new InstantCommand(
            pistons::extend,
            pistons),
          new InstantCommand(
            claw::intake,
            claw
          )
        )
      ).whileFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              pistons::retract, 
              pistons),
            new InstantCommand(
              claw::stop,
              claw
            ))
          );

      mOperator.rightBumper().whileTrue(
        new InstantCommand(
          claw::outtake,
          claw
        )
      ).whileFalse(
        new InstantCommand(
          claw::stop,
          claw
        )
      );

      mOperator.leftBumper().whileTrue(
        new InstantCommand(
          claw::intake,
          claw
        )
      ).whileFalse(
        new InstantCommand(
          claw::stop,
          claw
        )
      );

      mOperator.leftTrigger().whileTrue(
        new InstantCommand(
          arm::retract,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );
      
      mOperator.rightTrigger().whileTrue(
        new InstantCommand(
          arm::extend,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );

      //Tieu-Tam Controls
      mTieuTam.y().onTrue(
        new InstantCommand(
          pistons::extend,
          pistons
        )
      );

      mTieuTam.x().onTrue(
        new InstantCommand(
          pistons::retract,
          pistons
        )
      );

      mTieuTam.leftTrigger().whileTrue(
        new InstantCommand(
          arm::retract,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );

      mTieuTam.rightTrigger().whileTrue(
        new InstantCommand(
          arm::extend,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );

      mTieuTam.b().whileTrue(
        new InstantCommand(
          drivetrain::resetGyro, 
          drivetrain
        )
      );

      //Liam controls
      mLiam.leftBumper().whileTrue(
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

      mLiam.rightBumper().whileTrue(
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

      mLiam.leftTrigger().whileTrue(
        new InstantCommand(
          pistons::retract,
          pistons
        )
      );

      mLiam.rightTrigger().whileTrue(
        new InstantCommand(
          pistons::extend,
          pistons
        )
      );

      mLiam.povUp().whileTrue(
        new InstantCommand(
          arm::extend,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );

      mLiam.povDown().whileTrue(
        new InstantCommand(
          arm::retract,
          arm
        )
      ).whileFalse(
        new InstantCommand(
          arm::stop,
          arm
        )
      );

      mLiam.start().whileTrue(
        new InstantCommand(
          pivot::ratchetDisable, 
          pivot)
      );

      mLiam.back().whileTrue(
        new InstantCommand(
          pivot::ratchetEnable, 
          pivot)
      );

      CommandScheduler.getInstance().setDefaultCommand(
        pivot,
        new RunCommand(
          () -> pivot.addToTargetAngle(deadbander.applyLinearScaledDeadband(-mLiam.getLeftY(), 0.1) * 0.2),
           pivot
        )
      );

      mLiam.x().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(-10)
        )
      );

      mLiam.b().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(30)
        )
      );

      mLiam.a().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(50)
        )
      );
      mLiam.y().whileTrue(
        new InstantCommand(
          () -> pivot.changeSetpoint(15)
        )
      );

      }

      public void configureAutoChooser() {

        mAutoChooser.setDefaultOption("Nothing", new SequentialCommandGroup[]{null, null});

        mAutoChooser.addOption("TaxiCharge", new SequentialCommandGroup[] {
          new TaxiBlue(drivetrain),
          new TaxiRed(drivetrain)
        });

        SmartDashboard.putData("Auto Chooser", mAutoChooser);

      }

      public SequentialCommandGroup getAutonomousCommand() {
        
        int alliance = 0;
        if(DriverStation.getAlliance() == Alliance.Red){
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