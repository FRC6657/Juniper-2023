// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/*
 * Inacurate Mecanum "Sim" for Testing Purposes
 * Disclaimer "Will not be accurate to real hardware"
 */
public class MecanumSimulation {

    private TalonFXSimCollection mFrontLeftSimCollection;
    private TalonFXSimCollection mFrontRightSimCollection;
    private TalonFXSimCollection mBackLeftSimCollection;
    private TalonFXSimCollection mBackRightSimCollection;

    private BasePigeonSimCollection mBasePigeonSimCollection;

    private FlywheelSim mFrontLeftWheelSimulation;
    private FlywheelSim mFrontRightWheelSimulation;
    private FlywheelSim mBackLeftWheelSimulation;
    private FlywheelSim mBackRightWheelSimulation;

    private MecanumDriveKinematics mKinematics;

    private static double mGearRatio;

    private Supplier<MecanumDriveWheelSpeeds> mWheelSpeeds;
    private Supplier<double[]> mMotorSets;

    private static double dtSeconds = 0.02;

    public  MecanumSimulation(
        TalonFXSimCollection[] _motorSims,
        BasePigeonSimCollection _pigeonSim,
        double[] _sysid,
        MecanumDriveKinematics _kinematics,
        DCMotor _motor,
        double _gearRatio, 
        Supplier<MecanumDriveWheelSpeeds> _wheelSpeeds,
        Supplier<double[]> _motorSets
) {

        mFrontLeftSimCollection = _motorSims[0];
        mFrontRightSimCollection = _motorSims[1];
        mBackLeftSimCollection = _motorSims[2];
        mBackRightSimCollection = _motorSims[3];

        mBasePigeonSimCollection = _pigeonSim;

        LinearSystem<N1, N1, N1> _drivetrainPlant = LinearSystemId.identifyVelocitySystem(_sysid[1] / 12, _sysid[2] / 12);

        mFrontLeftWheelSimulation = new FlywheelSim(_drivetrainPlant, _motor, _gearRatio);
        mFrontRightWheelSimulation = new FlywheelSim(_drivetrainPlant, _motor, _gearRatio);
        mBackLeftWheelSimulation = new FlywheelSim(_drivetrainPlant, _motor, _gearRatio);
        mBackRightWheelSimulation = new FlywheelSim(_drivetrainPlant, _motor, _gearRatio);

        mKinematics = _kinematics;
        mGearRatio = _gearRatio;

        mWheelSpeeds = _wheelSpeeds;
        mMotorSets = _motorSets;
    }

    public void update() {

        mFrontLeftWheelSimulation.setInput(mMotorSets.get()[0] * RobotController.getBatteryVoltage());
        mFrontRightWheelSimulation.setInput(mMotorSets.get()[1] * RobotController.getBatteryVoltage());
        mBackLeftWheelSimulation.setInput(mMotorSets.get()[2] * RobotController.getBatteryVoltage());
        mBackRightWheelSimulation.setInput(mMotorSets.get()[3] * RobotController.getBatteryVoltage());

        mFrontLeftWheelSimulation.update(dtSeconds);
        mFrontRightWheelSimulation.update(dtSeconds);
        mBackLeftWheelSimulation.update(dtSeconds);
        mBackRightWheelSimulation.update(dtSeconds);

        double frontLeftVelocity = (mFrontLeftWheelSimulation.getAngularVelocityRPM() * mGearRatio * 2048) / 600;
        double frontLeftPositionDelta = frontLeftVelocity * 10  * dtSeconds;

        double frontRightVelocity = (mFrontRightWheelSimulation.getAngularVelocityRPM() * mGearRatio * 2048) / 600;
        double frontRightPositionDelta = frontRightVelocity * 10  * dtSeconds;

        double backLeftVelocity = (mBackLeftWheelSimulation.getAngularVelocityRPM() * mGearRatio * 2048) / 600;
        double backLeftPositionDelta = backLeftVelocity * 10  * dtSeconds;

        double backRightVelocity = (mBackRightWheelSimulation.getAngularVelocityRPM() * mGearRatio * 2048) / 600;
        double backRightPositionDelta = backRightVelocity * 10  * dtSeconds;

        mFrontLeftSimCollection.setIntegratedSensorVelocity((int)frontLeftVelocity);
        mFrontRightSimCollection.setIntegratedSensorVelocity((int)frontRightVelocity);
        mBackLeftSimCollection.setIntegratedSensorVelocity((int)backLeftVelocity);
        mBackRightSimCollection.setIntegratedSensorVelocity((int)backRightVelocity);
        
        mFrontLeftSimCollection.addIntegratedSensorPosition((int)frontLeftPositionDelta);
        mFrontRightSimCollection.addIntegratedSensorPosition((int)frontRightPositionDelta);
        mBackLeftSimCollection.addIntegratedSensorPosition((int)backLeftPositionDelta);
        mBackRightSimCollection.addIntegratedSensorPosition((int)backRightPositionDelta);

        mBasePigeonSimCollection.addHeading(Units.radiansToDegrees(mKinematics.toChassisSpeeds(mWheelSpeeds.get()).omegaRadiansPerSecond) * dtSeconds);

    }

}