/*
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.lang.*;

import static java.lang.Thread.sleep;

@Autonomous(name="Autonomous02", group="Iterative Opmode")  // @TeleOp(...) is the other common choice

public class Autonomous02 extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;

    public DcMotorController motor_controller_drive;
    public DcMotor motor_drive_left;
    public DcMotor motor_drive_right;

    // Initilization of drive train variables:
    //public double power_forward;
    public double power_back;
    //    public double power_RT;
//    public double power_LT;
    public double power_level;

    // Initilization of drive train variables:
    //public double power_forward;
    public double power_shooter;

    // Initialization of joystick buttons:
    public double button_RT;
    public double button_LT;

    public boolean button_RB;
    public boolean button_LB;
/*    public boolean button_a;
    public boolean button_y;
    public boolean button_b;
    public boolean button_x;*/

    //    public double joystick1_right_x;
    public double joystick1_right_y;
    public double joystick1_left_x;
    public double joystick1_left_y;
    public double joystick1_right_x;

    public double left_train_power;
    public double right_train_power;

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    public int n=0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");

        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");

        motor_drive_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_drive_right.setDirection(DcMotorSimple.Direction.FORWARD);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void loop() {
//        double instant = runtime.milliseconds();
//        while (instant > runtime.milliseconds() - 3000) {
//            updateTelemetry(telemetry);
//        }

        if (n==0) {
            driveFor(1, 1, 4000);
        }

        //telemetry.addData("Status", "Running: " + runtime.toString());


        //Vuforia Navigation Parameter Initializaiton:
        /*
        motor_drive_left.setPower(1);
        motor_drive_right.setPower(1);
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ASRQXlf/////AAAAGQ245EOVhUkkhQoi3eLqX09KdbhW+dBpEjCUrY/9xedC8aao+8Q0U0budcuCcNjOHmd6cgxcyS192sNXELz0KJDo0uS2PCZj1ddIToh6Nb6QwIbqnbwRa1CThS6lwVz8r2gda7SMWHfrtkvjHb/boVT6yVU27cTJ/uPq8iLKok103f93X/nXAZILO3tQ9/Ak944lXEhUJ51i/srWcs74gxKfv3y0CKu95B4TgIOd0Exgd24hswEwowrqnHI6p3gMDdQzqEOvk5lcgCIJ9F8H1VeyyLY25FVK9yx2hsv9tsR8B+ckIZAgrJd/axpk5+t4DvIBaDrTcVqFKUdtHUhWs2tmRp2VdhrhHnzkoU5I4pKA";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("wheels");
        beacons.get(1).setName("tools");
        beacons.get(2).setName("legos");
        beacons.get(3).setName("wheels");

        beacons.activate();

        for (VuforiaTrackable beac : beacons) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

            if(pose != null) {
                VectorF translation = pose.getTranslation();

                telemetry.addData(beac.getName() + "-Translation", translation);

                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                turnDegrees(degreesToTurn);

                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                updateTelemetry(telemetry);
            }
        }
        */
    }

    public void driveFor(double left_train_power, double right_train_power, int milliseconds) {
        n=1;
        motor_drive_left.setPower(left_train_power);
        motor_drive_right.setPower(-right_train_power);
//        double instant = runtime.milliseconds();
//        while (instant > runtime.milliseconds() - milliseconds) {
//            updateTelemetry(telemetry);
//        }

        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        motor_drive_left.setPower(0);
        motor_drive_right.setPower(0);


    }


    public void turnDegrees(double degrees) {
        double dDegree = (Math.abs(degrees));
        if (degrees < 0) {
            driveFor(-1, 1, (int) (75 * dDegree));
        } else if (degrees > 0) {
            driveFor(1, -1, (int) (75 * dDegree));
        } else {
            updateTelemetry(telemetry);
        }
    }
}