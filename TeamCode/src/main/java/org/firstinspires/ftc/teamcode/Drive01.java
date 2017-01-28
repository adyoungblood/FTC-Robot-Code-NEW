/*
*/
package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;

//import java.text.SimpleDateFormat;
//import java.util.Date;
@Disabled
@TeleOp(name="Drive01", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class Drive01 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorController motor_controller_shooter;
    public DcMotor shooter_motor_1;
    public DcMotor intake_motor_1;
    public DcMotor intake_motor_2;

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

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        //motor_controller_shooter = hardwareMap.dcMotorController.get("Motor_Controller_Shooter");
        //shooter_motor_1 = hardwareMap.dcMotor.get("Motor_Shooter_1");
        //intake_motor_1 = hardwareMap.dcMotor.get("Motor_Intake_1");
        //intake_motor_2 = hardwareMap.dcMotor.get("Motor_Intake_2");
        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
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

        drive_control();
        //shooter_control();
        //intake_control();

        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-gamepad1.right_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void shooter_control() {

        /*
        joystick2_right_y = gamepad2.right_stick_y;

        if (joystick2_right_y > 0.3) {
            intake_motor_1.setPower(-joystick2_right_y);
        }
        if (joystick2_right_y < -0.3) {
            intake_motor_1.setPower(0.15);
        }

        if (joystick2_right_y >= -0.3 && joystick2_right_y <= 0.3) {
            intake_motor_1.setPower(0);
        }


        if (gamepad2.start) {
            intake_motor_1.setPower(0);
        }
        */
    }


//******************************************************************************
//------------------------------------------------------------------------------
//Routine for Driving Robot with Front Buttons:

    public void drive_control() {

        // Define mode in which power is controlled:
        // Control with triggers
        /*
        button_RT = gamepad1.right_trigger * power_level;
        button_RB = gamepad1.right_bumper;

        if (button_RT > 0) {
            motor_drive_right.setPower(-button_RT);
        } else if (button_RB) {
            motor_drive_right.setPower(power_back);
        } else {
            motor_drive_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coasting
            motor_drive_right.setPower(0);
        }

        button_LT = gamepad1.left_trigger * power_level;
        button_LB = gamepad1.left_bumper;

        if (button_LT > 0) {
            motor_drive_left.setPower(button_LT);
        } else if (button_LB) {
            motor_drive_left.setPower(-power_back);
        } else {
            motor_drive_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coasting
            motor_drive_left.setPower(0);
        }
        */
        // Joystick control
        //joystick1_left_x = gamepad1.left_stick_x;
        //joystick1_right_x = -gamepad1.right_stick_x;
        joystick1_left_y = gamepad1.left_stick_y;
        joystick1_right_y = gamepad1.right_stick_y;

        motor_drive_left.setPower(joystick1_left_y);
        motor_drive_right.setPower(joystick1_right_y);
    }

    public void intake_control() {

        button_LT = gamepad1.left_trigger;
        button_LB = gamepad1.left_bumper;

        if (button_LB) {
            intake_motor_1.setPower(button_LT);
            intake_motor_2.setPower(button_LT);
        } else {
            intake_motor_1.setPower(-button_LT);
            intake_motor_2.setPower(-button_LT);
        }

        /*
        joystick1_left_y = gamepad1.left_stick_y;

        if (joystick1_left_y > 0.3) {
            intake_motor_1.setPower(joystick1_left_y);
        }
        if (joystick1_left_y < -0.3) {
            intake_motor_1.setPower(joystick1_left_y);
        }

        if (joystick1_left_y >= -0.3 && joystick1_left_y <= 0.3) {
            intake_motor_1.setPower(0);
        }
        */
    }


}