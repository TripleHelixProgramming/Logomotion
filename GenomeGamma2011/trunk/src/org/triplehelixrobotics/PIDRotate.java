/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package org.triplehelixrobotics;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 *
 * @author matthew.lythgoe
 */
public class PIDRotate implements PIDOutput
{
    private RobotDrive rDrive;
    private double forward;

    public PIDRotate(RobotDrive drive, double speed)
    {
        forward = speed; // sets the intended forward speed
        rDrive = drive;
    }

    public void pidWrite(double output)
    {
        rDrive.tankDrive(forward + output, forward - output); // modifies the forward outputs of the motors to turn
    }
}
