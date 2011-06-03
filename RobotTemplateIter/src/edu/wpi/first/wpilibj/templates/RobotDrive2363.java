/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;


/**
 *
 * @author matthew.lythgoe
 */
public class RobotDrive2363 extends RobotDrive
{
    private static final double DRIVE_P = 0;
    private static final double DRIVE_I = 0;
    private static final double DRIVE_D = 0;

    private Gyro gyro;

    public RobotDrive2363(int frontLeftMotor, int rearLeftMotor, int frontRightMotor, int rearRightMotor)
    {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }
    
    public void setGyro(int channel)
    {
        gyro = new Gyro(channel);
    }

    public void forward(double speed, double time)
    {
        PIDRotate  rotate = new PIDRotate(this, speed);
        PIDController driveForward = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D, gyro, rotate);
        driveForward.setTolerance(0.05); // sets the tolerance to withing 5% of the setpoint
        driveForward.setSetpoint(gyro.getAngle()); // sets the setpoint to the current heading
        //driveForward.setContinuous(); // write PIDInput to limit range from 0-359
        driveForward.enable(); // starts the PIDController
        
        Timer.delay(time);
        
        driveForward.disable();
    }

    public void turn(double speed, int angle)
    {
        PIDRotate  rotate = new PIDRotate(this, 0);
        PIDController turn = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D, gyro, rotate);
        turn.setTolerance(0.05); // sets the tolerance to withing 5% of the setpoint
        turn.setSetpoint(angle); // sets the setpoint to the intended angle
        turn.setContinuous(); // sets the PID to wor on a continuous 360 circle
        turn.enable(); // starts the PIDController

        while(!turn.onTarget()) // turn until the robot is at the specified angle
        {
            //keep turning
        }
        turn.disable(); // stops the PIDController
    }

}
