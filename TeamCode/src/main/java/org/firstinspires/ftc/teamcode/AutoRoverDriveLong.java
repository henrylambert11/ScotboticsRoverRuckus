package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="Auto Rover Drive Longer", group="Autononmous")

public class AutoRoverDriveLong extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final int Sample = 1;
    static final int Land = 0;
    static final int DRIVE_TO_BOX = 2;
    static final int STOPROBOT = 3;

    int state = Land;

    DcMotor Right;
    DcMotor Left;
    DcMotor Collector;
    DcMotor Lift;
    DcMotor Extender;
    DcMotor Pull;

    ColorSensor colorSensorFront;
    ColorSensor colorSensorBack;


    Servo Bucket1;
    Servo Bucket2;

    Servo Hold;

    float hsvValues[] = {0F, 0F, 0F};
    double hold = 1;
    double turn = 1;
    boolean LEDState = true;


    long drive_time = 1800;
    long turn_time = 3500;


    final float values[] = hsvValues;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;


    @Override
    public void runOpMode() {

        Right        = hardwareMap.dcMotor.get("rightDrive"); //right motor
        Left         =  hardwareMap.dcMotor.get("leftDrive"); //left
        Lift = hardwareMap.dcMotor.get("lift"); //up lift

        colorSensorFront = hardwareMap.colorSensor.get("sensor_color_front");
        colorSensorBack = hardwareMap.colorSensor.get("sensor_color_back");

        Bucket1 = hardwareMap.servo.get("Bucket1");//Bucket lift
        Bucket2 = hardwareMap.servo.get("Bucket2");//Bucket lift
        Hold = hardwareMap.servo.get("hold");//stops lift

        colorSensorFront.enableLed(LEDState);


        waitForStart();
        while (opModeIsActive()) {

            switch (state) {
                case Land:
                    land();
                    break;
                //  case VUFORIA:
                //    vuforia();
                //  break;
                case Sample:
                    sample();
                    break;
                case DRIVE_TO_BOX:
                    drive_to_box();
                    break;
                case STOPROBOT:
                    stoprobot();
                    break;

            }

        }
    }

    public void vuforia() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AaKj3k//////AAAAGRCFygr30kHThYvTzaUV3+UUSXBWuZOJcwZShnaBSZYsn5iIDMIrNAoav2sF9qS9YTm2xtJ3Ox8UgNgv55hq4+siLP7M7+BYUTBFq2Azvvg88gtUrAOtv+ER0gLpR85Qt+BGULSIS5p7ksf4T/CnhkeLty569q1mvCKIkkYBMBHW1yLm+IlO3ZaJSqfN+WLwN5bMAMSkU7pd7ZCl2gLscLDo7IlfcJCCLopGm9HQx8ltLoMzRaErO3YA9o/rP7wOd56s37f1D3V5f8uEMYP13eyyctteM8WjXpqlEnoOymJDX/7FGx/rq4ZUn/NfGUh8xjEPWqc3eNGOnNWIYqveKbdBGRKtV+yUsOExiu1LdLDh";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void land() {

        /*

        Hold.setPosition(0);
        sleep(1000);

        stop_drive(100);
        sleep(100);

        Lift.setPower(-1);
        sleep(1000);

        stop_drive(500);
        sleep(500);

        Lift.setPower(1);
        sleep(100);

        turn_right(1000);
        sleep(1000);

        stop_drive(100);
        sleep(100);

        */

        state = Sample;

    }

    public void sample() {

        drive_forward(450);
        sleep(450);

        stop_drive(300);
        sleep(300);

        turn_right(1600);
        sleep(1600);

        stop_drive(300);
        sleep(300);

        drive_forward(500);
        sleep(500);

        stop_drive(300);
        sleep(300);

        turn_right(850);
        sleep(850);

        stop_drive(100);
        sleep(100);

        drive_forward(1700);
        sleep(1700);

        stop_drive(1000);
        sleep(1000);
        state = DRIVE_TO_BOX;

    }

    public void drive_to_box() {

            Bucket1.setPosition(1);
            Bucket2.setPosition(-1);

            sleep(2000);

            Bucket1.setPosition(0);
            Bucket2.setPosition(0);

            state = STOPROBOT;

    }

    //stoping the robot at the end of the program
    public void stoprobot() {

        stop_drive(1000000000);
        sleep(1000000000);

    }
    //drive backward method
    void drive_backward(double time) {
        Left.setPower(-hold);
        Right.setPower(hold);
    }
    //drive forward method
    void drive_forward(double time) {
        Left.setPower(hold);
        Right.setPower(-hold);
    }
    //turn left method
    void turn_left(double time) {
        Left.setPower(turn);
        Right.setPower(turn);
    }
    //turn right method
    void turn_right(double time) {
        Left.setPower(-turn);
        Right.setPower(-turn);
    }
    //stop robot method
    void stop_drive(double time) {
        Left.setPower(0.0);
        Right.setPower(0.0);
    }
}


