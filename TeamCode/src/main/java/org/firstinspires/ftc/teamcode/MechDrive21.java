//package that allows importing into the phone
package org.firstinspires.ftc.teamcode;

//import a lot of different functions, methods and commands for the ftc hardware

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TELEOP", group="TeleOp")
public class MechDrive21 extends OpMode {

    public DcMotor aDrive;
    public DcMotor cDrive;
    public DcMotor bDrive;
    public DcMotor dDrive;
    public DcMotor Arm;
    public DcMotor ArmExtender;
    public DcMotor Spindle;
    public DcMotor Lift;
    public Servo Cover;

    public void init() {
        //list of the all the motors and connect themselves to the actual motors
        //the things in parantheses ex. "frontLeft" are what the motors are named in the configuration on the robot controller phone
        aDrive = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive = hardwareMap.get(DcMotor.class, "dDrive");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        ArmExtender = hardwareMap.get(DcMotor.class, "ArmExtender");
        Spindle = hardwareMap.get(DcMotor.class, "Spindle");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Cover = hardwareMap.get(Servo.class, "Cover");

        cDrive.setDirection(DcMotor.Direction.REVERSE);
        dDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // Driving controls

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        aDrive.setPower(v1);
        cDrive.setPower(v2);
        bDrive.setPower(v3);
        dDrive.setPower(v4);

        // Lifter controls

        if (gamepad1.a) {
            Lift.setPower(1);
        }
        if (gamepad1.y) {
            Lift.setPower(-1);
        }
        if (!gamepad1.a && !gamepad1.y) {
            Lift.setPower(0);
        }

        // Arm Controls

        Arm.setPower(gamepad2.right_stick_y);

        // Arm Extender Controls

        ArmExtender.setPower(gamepad2.left_stick_y);

        // Spindle Controls

        if(gamepad1.left_bumper) {
            Spindle.setPower(1);
        }
        if(gamepad1.right_bumper) {
            Spindle.setPower(-1);
        }
        if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
            Spindle.setPower(0);
        }

        // Cover Controls

        if(gamepad2.right_bumper) {
            Cover.setPosition(1);
        }
        if(gamepad2.left_bumper) {
            Cover.setPosition(0);
        }
        if(!gamepad2.right_bumper && !gamepad2.left_bumper) {
            Cover.setPosition(0.5);
        }

    }

    public void stop() {
        //nothing here? probably gotta call garbage collection at some point
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
