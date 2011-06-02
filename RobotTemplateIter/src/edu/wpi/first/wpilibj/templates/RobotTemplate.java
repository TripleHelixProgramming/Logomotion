/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot
{
    private static final double ELEVATOR_POSITION_PICKUP = 0;
    private static final double ELEVATOR_POSITION_STOW = 0;
    private static final double ELEVATOR_POSITION_1 = 0;
    private static final double ELEVATOR_POSITION_2 = 0;
    private static final double ELEVATOR_POSITION_3 = 0.969;
    private static final double ELEVATOR_POSITION_4 = 1.351;
    private static final double ELEVATOR_POSITION_5 = 2.65;
    private static final double ELEVATOR_POSITION_6 = 3;
    private static final double ARM_POSITION_PICKUP = -1.429;
    private static final double ARM_POSITION_STOW = -0.1875;
    private static final double ARM_POSITION_1 = -0.6328;
    private static final double ARM_POSITION_2 = -0.5;
    private static final double ARM_POSITION_3 = -0.35;
    private static final double ARM_POSITION_4 = -0.35;
    private static final double ARM_POSITION_5 = -0.25;
    private static final double ARM_POSITION_6 = -0.25;
    private static final int LEFT_FRONT_JAG_PWM = 2;
    private static final int RIGHT_FRONT_JAG_PWM = 3;
    private static final int LEFT_REAR_JAG_PWM = 4;
    private static final int RIGHT_REAR_JAG_PWM = 5;
    private static final int ARM_CAN_ADDRESS = 6;
    private static final int ELEVATOR_CAN_ADDRESS = 7;
    private static final int ENCODER_REVS = 64;
    private static final double ARM_P = 350;
    private static final double ARM_I = 0;
    private static final double ARM_D = 0;
    private static final double ELEVATOR_P = 700;
    private static final double ELEVATOR_I = 0;
    private static final double ELEVATOR_D = 0;
    private static final int UPPER_GRIPPER_OPEN_CHANNEL = 1;
    private static final int UPPER_GRIPPER_CLOSED_CHANNEL = 2;
    private static final int LOWER_GRIPPER_OPEN_CHANNEL = 3;
    private static final int LOWER_GRIPPER_CLOSED_CHANNEL = 4;
    private static final int MINIBOT_DEPLOY_OUT_CHANNEL = 5;
    private static final int MINIBOT_DEPLOY_IN_CHANNEL = 6;
    private static final int COMPRESSOR_SLOT = 4;
    private static final int COMPRESSOR_CHANNEL = 4;
    private static final int RELAY_SLOT = 4;
    private static final int RELAY_CHANNEL = 1;
    private static final int RIGHT_TOWER_SENSOR_CHANNEL = 3;
    private static final int LEFT_TOWER_SENSOR_CHANNEL = 4;
    private static final double CENTER_P = 0;
    private static final double CENTER_I = 0;
    private static final double CENTER_D = 0;

    private RobotDrive2363 rDrive;
    private Joystick j;
    private Jaguar lfJag;// = new CANJaguar(2);
    private Jaguar lrJag;// = new CANJaguar(3);
    private Jaguar rfJag;// = new CANJaguar(4);
    private Jaguar rrJag;// = new CANJaguar(5);
    private CANJaguar armJag;// = new CANJaguar(6);
    private CANJaguar eleJag;// = new CANJaguar(7);

    private Compressor comp;
    private Solenoid uGripOpen;
    private Solenoid uGripClose;
    private Solenoid lGripOpen;
    private Solenoid lGripClose;
    private Solenoid miniBotIN;
    private Solenoid miniBotOUT;

    private AnalogChannel rightTowerSensor;
    private AnalogChannel leftTowerSensor;

    PIDController center;

    //private PIDController armPID;
    //private PIDController elePID;

    //private Encoder armEncoder;
    //private Encoder eleEncoder;

    private double x;
    private double y;
    private double z;
    private PIDPoleCenter poleInput;
    private PIDRotate poleRotate;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        x = 0;
        y = 0;
        z = 0;

        j = new Joystick(1);

        // sets the channels for the compressor and relay spike
        comp = new Compressor(COMPRESSOR_SLOT, COMPRESSOR_CHANNEL, RELAY_SLOT, RELAY_CHANNEL); // sets the compressor and relay slots and channels
                // starts the compressor
        comp.start();
        
        // sets the driv jaguars to their pwm channels and the joystick
        lfJag = new Jaguar(LEFT_FRONT_JAG_PWM);
        lrJag = new Jaguar(LEFT_REAR_JAG_PWM);
        rfJag = new Jaguar(RIGHT_FRONT_JAG_PWM);
        rrJag = new Jaguar(RIGHT_REAR_JAG_PWM);

        try
        {
            // sets up the arm CANJaguar
            armJag = new CANJaguar(ARM_CAN_ADDRESS, CANJaguar.ControlMode.kPosition);
            armJag.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            armJag.configEncoderCodesPerRev(ENCODER_REVS); // number of ticks on encoder
            armJag.setPID(ARM_P, ARM_I, ARM_D);
            armJag.configSoftPositionLimits(1, 0); // set max and min limits
            armJag.enableControl(); // start closed loop control
            armJag.setX(ARM_POSITION_STOW);  // set the start position to stow

            // sets up the elevator CANJaguar
            eleJag = new CANJaguar(ELEVATOR_CAN_ADDRESS, CANJaguar.ControlMode.kPosition);
            eleJag.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            eleJag.configEncoderCodesPerRev(ENCODER_REVS); // number of ticks on encoder
            eleJag.setPID(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
            eleJag.configSoftPositionLimits(1, 0); // set max and min limits
            eleJag.enableControl(); // start closed loop control
            eleJag.setX(ELEVATOR_POSITION_STOW); // set the start position to stow
        } 
        catch (CANTimeoutException ex)
        {
            ex.printStackTrace();
        }

        // sets up the motor drive
        rDrive = new RobotDrive2363(LEFT_FRONT_JAG_PWM, LEFT_REAR_JAG_PWM, RIGHT_FRONT_JAG_PWM,  RIGHT_REAR_JAG_PWM);
//        rDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
//        rDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
//        rDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
//        rDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);

        // sets up the sensors and the pid values for the minibot tower centering
        rightTowerSensor = new AnalogChannel(RIGHT_TOWER_SENSOR_CHANNEL);
        leftTowerSensor = new AnalogChannel(LEFT_TOWER_SENSOR_CHANNEL);
        poleInput = new PIDPoleCenter(leftTowerSensor, rightTowerSensor);
        poleRotate = new PIDRotate(rDrive, 0.3);
        center = new PIDController(CENTER_P, CENTER_I, CENTER_D, poleInput, poleRotate);


//        armEncoder = new Encoder(1, 2);
//        eleEncoder = new Encoder(3, 4);

//        armPID = new PIDController(0, 0, 0, armEncoder, armJag);
//        armPID.setOutputRange(-1.0, 1.0); // sets the range of the  output to the Jaguar
//        armPID.setInputRange(0, 1.0); // sets the range of the input from the encoder
//        armPID.setTolerance(0.05); // sets the accuracy tolerance to 5%
//        armPID.enable();
//        elePID = new PIDController(0, 0, 0, eleEncoder, eleJag);
//        elePID.setOutputRange(-1, 1.0); // sets the range of the  output to the Jaguar
//        elePID.setInputRange(0, 1.0); // sets the range of the input from the encoder
//        elePID.setTolerance(0.05); // sets the accuracy tolerance to 5%
//        elePID.enable();

        // sets up the gripper solenoids and sets their default values to close the gripper
        uGripOpen = new Solenoid(UPPER_GRIPPER_OPEN_CHANNEL);
        uGripOpen.set(false);
        uGripClose = new Solenoid(UPPER_GRIPPER_CLOSED_CHANNEL);
        uGripClose.set(true);
        lGripOpen = new Solenoid(LOWER_GRIPPER_OPEN_CHANNEL);
        lGripOpen.set(false);
        lGripClose = new Solenoid(LOWER_GRIPPER_CLOSED_CHANNEL);
        lGripClose.set(true);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
//        armEncoder.start();
//        eleEncoder.start();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic()
    {   
        try
        {
            setButtonAction(); // checks the controller button presses and performs the appropriate action
        }
        catch (CANTimeoutException ex)
        {}
    }

    public void teleopContinuous()
    {
        x = j.getX(); // sets the horizontal drive value
        y = j.getY(); // sets the forward drive value
        z = j.getZ(); // sets the drive rotation
        
        rDrive.mecanumDrive_Cartesian(x, y, z, 0); // sets the drive values
    }

    private void setButtonAction() throws CANTimeoutException
    {
        int action = -1;
        for(int i = 0; i < 16; i++) // finds the first pressed button
            if(m_ds.getDigitalIn(i))
                action = i;

        switch(action) // sets the position for the arm and elevator depending on which button was pressed
        {
            case 0: // pick-up position/open upper gripper and close lower gripper
                armJag.setX(ARM_POSITION_PICKUP);
                eleJag.setX(ELEVATOR_POSITION_PICKUP);
                uGripOpen.set(true);
                uGripClose.set(false);
                lGripOpen.set(false);
                lGripClose.set(true);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 1: // rack 1 position
                armJag.setX(ARM_POSITION_1);
                eleJag.setX(ELEVATOR_POSITION_1);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 2: // rack 2 position
                armJag.setX(ARM_POSITION_2);
                eleJag.setX(ELEVATOR_POSITION_2);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 3: // rack 3 position
                armJag.setX(ARM_POSITION_3);
                eleJag.setX(ELEVATOR_POSITION_3);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 4: // rack 4 position
                armJag.setX(ARM_POSITION_4);
                eleJag.setX(ELEVATOR_POSITION_4);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 5: // rack 5 position
                armJag.setX(ARM_POSITION_5);
                eleJag.setX(ELEVATOR_POSITION_5);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 6: // rack 6 position
                armJag.setX(ARM_POSITION_6);
                eleJag.setX(ELEVATOR_POSITION_6);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 7: // stow position
                armJag.setX(ARM_POSITION_STOW);
                eleJag.setX(ELEVATOR_POSITION_STOW);
//                elePID.setSetpoint(0);
//                armPID.setSetpoint(0);
                break;
            case 8: // pick up button - close upper
                uGripOpen.set(false);
                uGripClose.set(true);
                break;
            case 9: // drop button - open lower
                lGripOpen.set(true);
                lGripClose.set(false);
                break;
            case 10: // pick up button inverse - open upper
                uGripOpen.set(true);
                uGripClose.set(false);
                break;
            case 11: // drop up button - close lower
                lGripOpen.set(false);
                lGripClose.set(true);
                break;
            case 12: // minibot in
                miniBotIN.set(true);
                miniBotOUT.set(false);
                break;
            case 13: // minibot out
                miniBotIN.set(false);
                miniBotOUT.set(true);
                break;
            case 14: // center on pole
                center.enable();
                break;
            default: // no buttons pressed
                center.disable();
                break;
                //do nothing
        }
    }

}
