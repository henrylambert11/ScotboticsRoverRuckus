
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoSample", group="Autononmous")
public class AutoSample_Crater extends LinearOpMode {

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

    //VUFORIA STUFF
    private GoldAlignDetector detector;         //Makes the phone look for the gold
    double goldPos = 0;                         //The angle the gold is away from the robot

    //TIMERS
    private ElapsedTime runtime = new ElapsedTime();        //Tells us when the 30 seconds is up

    @Override
    public void runOpMode() {

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

        //Vuforia
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Phone Stuff
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral

        goldPos = detector.getXPosition();  // Gets gold block posistion

        sleep(1000);            // Waits to make sure it is correct

        // Move Depending on gold position
        if(goldPos < 160){
            driftleft();
            sleep(1500);
        }else if (goldPos > 500) {
            driftright();
            sleep(1500);
        }else{
            forward();
        }

        forward();                  //Drives into crater
        sleep(4000);

        detector.disable();

    }

    //Mechanum Wheel Commands

    public void turnLeft(){
        telemetry.addData("status" , "turnLeft");
        telemetry.update();

        aDrive.setPower(-1);
        bDrive.setPower(-1);
        cDrive.setPower(1);
        dDrive.setPower(1);

    }

    public void turnRight(){
        telemetry.addData("status" , "turnRight");
        telemetry.update();

        aDrive.setPower(1);
        bDrive.setPower(1);
        cDrive.setPower(-1);
        dDrive.setPower(-1);

    }

    public void forward(){
        telemetry.addData("status" , "forward");
        telemetry.update();

        aDrive.setPower(1);
        bDrive.setPower(1);
        cDrive.setPower(1);
        dDrive.setPower(1);

    }

    public void backward(){
        telemetry.addData("status" , "forward");
        telemetry.update();

        aDrive.setPower(-1);
        bDrive.setPower(-1);
        cDrive.setPower(-1);
        dDrive.setPower(-1);

    }

    public void driftleft(){
        telemetry.addData("status" , "forward");
        telemetry.update();

        aDrive.setPower(-1);
        bDrive.setPower(1);
        cDrive.setPower(1);
        dDrive.setPower(-1);

    }

    public void driftright(){
        telemetry.addData("status" , "forward");
        telemetry.update();

        aDrive.setPower(1);
        bDrive.setPower(-1);
        cDrive.setPower(-1);
        dDrive.setPower(1);

    }
       public void encoderDrive(double speed, double aInches, double bInches, double cInches, double dInches) {
        // 4 inch pinion diameter
        double ad = 720 * (aInches / 12.57);
        int aDistance = (int) ad;
        double bd = 720 * (bInches / 12.57);
        int bDistance = (int) bd;
        double cd = 720 * (cInches / 12.57);
        int cDistance = (int) cd;
        double dd = 720 * (dInches / 12.57);
        int dDistance = (int) dd;

        aDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        aDrive.setTargetPosition(aDistance);
        bDrive.setTargetPosition(bDistance);
        cDrive.setTargetPosition(cDistance);
        dDrive.setTargetPosition(dDistance);
        aDrive.setPower(speed);
        bDrive.setPower(speed);
        cDrive.setPower(speed);
        dDrive.setPower(speed);

        while (aDrive.isBusy() && bDrive.isBusy() && cDrive.isBusy() && dDrive.isBusy() && opModeIsActive()) {
            idle();
        }
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
    }
}

