/*
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

//import java.text.SimpleDateFormat;
//import java.util.Date;

@TeleOp(name="TestMode", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class TestMode extends OpMode {
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
    public double joystick2_right_y;


    public double left_train_power;
    public double right_train_power;

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    OpticalDistanceSensor ods1;
    DeviceInterfaceModule CDI;

    //sensor value between 0 and 1023
    double raw1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        //shooter_motor_1 = hardwareMap.dcMotor.get("Motor_Shooter_1");
        intake_motor_1 = hardwareMap.dcMotor.get("Motor_Intake_1");
        //motor_drive_left = hardwareMap.dcMotor.get("Motor_Drive_Left");
        //motor_drive_right = hardwareMap.dcMotor.get("Motor_Drive_Right");

        //power_level = 1.0;
        //power_back = 0.7;


        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");

        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

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
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        if (gamepad1.a) {
            double instant = runtime.seconds();
            telemetry.addData("Last Captured Instant:", instant);
            while (instant > runtime.seconds() - 1) {
                driveFor(1, -1, 1);
            }
            //Waits one second and drives while it does that without sleeping
        }

        raw1 = ods1.getLightDetected() * 1023;

        telemetry.addData("ODS1", raw1);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void driveFor(int left_train_power, int right_train_power, int seconds) {
        motor_drive_left.setPower(left_train_power);
        motor_drive_right.setPower(-right_train_power);
        try {
            Thread.sleep(seconds * 1000);
            motor_drive_left.setPower(0);
            motor_drive_right.setPower(0);
            /*
            if (gamepad1.start) {
                Thread.currentThread().interrupt();
            }
            */
            // Somehow make this check no matter what is happening
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}