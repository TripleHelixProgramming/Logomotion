/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.PIDSource;

/**
 *
 * @author matthew.lythgoe
 */
public class PIDPoleCenter implements PIDSource
{
    private AnalogChannel left;
    private AnalogChannel right;

    public PIDPoleCenter(AnalogChannel aLeft, AnalogChannel aRight)
    {
        left = aLeft; // sets the left sensor
        right = aRight; // sets the right sensor
    }
    public double pidGet()
    {
        return left.getValue() - right.getValue(); // gets the direction and distance  to rotate the robot
    }

}
