// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;
import frc.utils.Conversions;

public class Drivetrain extends SubsystemBase {
    /* Motor initialization */
    private WPI_VictorSPX mLeftMotor0;
    private WPI_VictorSPX mLeftMotor1;
    private WPI_VictorSPX mRightMotor0;
    private WPI_VictorSPX mRightMotor1;
    //fart

    private DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);

    private PIDController mLeftPIDController = new PIDController(Constants.Drivetrain.LEFT_KP, 0, 0);
    private PIDController mRightPIDController = new PIDController(Constants.Drivetrain.RIGHT_KP, 0, 0);

    public Drivetrain() {
        mLeftMotor0 = new WPI_VictorSPX(Constants.Drivetrain.LEFT_MOTOR_0_ID);
        mLeftMotor1 = new WPI_VictorSPX(Constants.Drivetrain.LEFT_MOTOR_1_ID);
        mRightMotor0 = new WPI_VictorSPX(Constants.Drivetrain.RIGHT_MOTOR_0_ID);
        mRightMotor1 = new WPI_VictorSPX(Constants.Drivetrain.RIGHT_MOTOR_1_ID);
    }

    /**
     * Public method to control Drivetrain
     * 
     * @param leftJoystickValue move robot back and forth
     * @param rightJoystickValue turn robot (rad/s)
     */
    // 
    public void Drive(double leftJoystickValue, double rightJoystickValue){
        DifferentialDriveWheelSpeeds wheelSpeeds = mKinematics.toWheelSpeeds(new ChassisSpeeds(leftJoystickValue, 0.0, rightJoystickValue));

        setSpeeds(wheelSpeeds);
    }

    /**
     * Private method to set the speeds of our drivetrain or something like that idk i used bing ai trust ong ong
     * 
     * @param wheelSpeeds differnetdrivewheelspeeds object that gives
     */

    private void setSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        double leftOutput = mLeftPIDController.calculate(Conversions.encoder_ticks_to_rpm(mLeftMotor0.getSelectedSensorVelocity(), Constants.Drivetrain.gearratio), wheelSpeeds.leftMetersPerSecond);
        double rightOutput = mRightPIDController.calculate(Conversions.encoder_ticks_to_rpm(mLeftMotor0.getSelectedSensorVelocity(), Constants.Drivetrain.gearratio), wheelSpeeds.rightMetersPerSecond);

        mLeftMotor0.setVoltage(leftOutput);
        mLeftMotor1.setVoltage(leftOutput);
        mRightMotor0.setVoltage(rightOutput);
        mRightMotor1.setVoltage(rightOutput);
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
