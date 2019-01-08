//package that allows importing into the phone
package org.firstinspires.ftc.teamcode;

//import a lot of different functions, methods and commands for the ftc hardware

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TELEOP", group="TeleOp")
public class MechDrive21 extends OpMode {

    /* Declare OpMode members. */

    // ALL THE MOTORS ON THE DRIVE SYSTEM
    private DcMotor aDrive = null;              //Front Left
    private DcMotor bDrive = null;              //Back Left
    private DcMotor cDrive = null;              //Front right
    private DcMotor dDrive = null;              //Back right

    //FUNCTIONS MOTORS
    public DcMotor Arm;                         //Moves the arm up and down
    public DcMotor ArmExtender;                 //Makes the arm grow in length
    public DcMotor Spindle;                     //Spins the collection spindle
    public DcMotor Lift;                        //Gets the robot on and off the lander

    //SERVOS
    public Servo Cover;                         //Covers the box that the minerals are in


    public void init() {

        //PUTS THE NAMES OF THE MOTORS INTO THE CODE

        //DRIVE Motors
        aDrive = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive = hardwareMap.get(DcMotor.class, "dDrive");
        cDrive.setDirection(DcMotor.Direction.REVERSE);
        dDrive.setDirection(DcMotor.Direction.REVERSE);

        //FUNCTIONS Motors
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
        Spindle = hardwareMap.get(DcMotor.class, "Spindle");
        Lift = hardwareMap.get(DcMotor.class, "Lift");

        //SERVOS
        Cover = hardwareMap.get(Servo.class, "Cover");
    }

    @Override
    public void loop() {

        // DRIVING CONTROLS

        // Gives the stick special values for the mechanum wheel drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        //  Assigns a variable the motor power based off of special calculations
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //  Gives the motors power so they turn
        aDrive.setPower(v1);
        cDrive.setPower(v2);
        bDrive.setPower(v3);
        dDrive.setPower(v4);


        // LIFT CONTROLS

        if (gamepad1.a) {                       //If "A" is pressed then the robot will go up the lander
            Lift.setPower(1);
        }
        if (gamepad1.y) {                       //If "Y" is pressed then the robot will go down the lander
            Lift.setPower(-1);
        }
        if (!gamepad1.a && !gamepad1.y) {       //If neither are pressed then the motor will not have power
            Lift.setPower(0);
        }

        // ARM CONTROLS

        Arm.setPower(gamepad2.right_stick_y);           //Tells the arm to move up or down

        // ARM EXTENDER CONTROLS

        ArmExtender.setPower(gamepad2.left_stick_y);    //Tells the arm to increase or decrease in length

        // SPINDLE CONTROLS

        if(gamepad1.left_bumper) {      //If the left bumber is pressed we collect minerals
            Spindle.setPower(1);
        }
        if(gamepad1.right_bumper) {     //If the right bumper is pressed then we can eject minerals
            Spindle.setPower(-1);
        }
        if(!gamepad1.left_bumper && !gamepad1.right_bumper) {       //If nothing is pressed then nothing happens
            Spindle.setPower(0);
        }

        // COVER CONTROLS

        if(gamepad2.right_bumper) {     //Puts cover down
            Cover.setPosition(1);
        }
        if(gamepad2.left_bumper) {      //Lifts cover up
            Cover.setPosition(0);
        }
        if(!gamepad2.right_bumper && !gamepad2.left_bumper) {       // If nothing is pressed then the cover does not move
            Cover.setPosition(0.5);
        }

    }

    public void stop() {
        //tells everything to cut all power
        //All motors go to 0 power when cut, servos stop moving at 0.5 power (Mochi)
        aDrive.setPower(0);
        cDrive.setPower(0);
        bDrive.setPower(0);
        dDrive.setPower(0);

        Arm.setPower(0);
        ArmExtender.setPower(0);
        Lift.setPower(0);
        Spindle.setPower(0);

        Cover.setPosition(0.5);

    }

}

/*
        Driving priorties list

        Mech Drive
        Arm
        Lift
        Extending arm
        Spindle for collection
        Cover

 */
