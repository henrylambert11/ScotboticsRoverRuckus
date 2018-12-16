
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="AutoSample_Linear", group="Chris")
public class AutoSample_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor aDrive = null;
    private DcMotor bDrive = null;
    private DcMotor cDrive = null;
    private DcMotor dDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    private GoldAlignDetector detector;

    double goldPos = 0;


    @Override
    public void runOpMode() {

        //Motors
        aDrive = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive = hardwareMap.get(DcMotor.class, "dDrive");
        cDrive.setDirection(DcMotor.Direction.REVERSE);

        dDrive.setDirection(DcMotor.Direction.REVERSE);


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

        goldPos = detector.getXPosition();

        sleep(1000);

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




        forward();
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
}

