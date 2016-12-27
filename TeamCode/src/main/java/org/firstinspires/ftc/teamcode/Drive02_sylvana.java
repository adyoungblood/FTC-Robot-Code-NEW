package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorController;

//import java.text.SimpleDateFormat;
//import java.util.Date;
/* (V 2.1 of drivetrain program)
 * Features:
 * Future proofed code for other, advanced functions (e.g. capping, pushing buttons, etc)
 * Basic program for Victanus's new drive train.
 * Basics:
 *  Forward - gamepad 2a, 1a = 1 (pressed) Motors left, right = 1
 * Turn right  - gamepad 2a, 1b = 1 (pressed) Motor right = -1, motor left = 1
 * Turn left  -  gamepad 2b, 1a = 1 (pressed) Motor right = 1, motor left = -1
 * Backward - gampad 2b, 1b = 1 (pressed) Motor right = -1, motor left = -1
 *  else - Motors left, right = 0
 *  Special Functions:
 *  Gamepad.x - button push - pushing x will push the button; default is retracted
 *  left stick y - capping - pushing upwards and downwards on the y-axis f controller will change
 *  the height of the capping mechanism
 *  right stick y - intake mechanism - pushing upwards
 *  Gamepad.b - shoot ball -
 *  <code that is commented out and indented will be added later>
 */
@TeleOp(name = "Drive02_sylvana", group = "Iterative Opmode")
class Drive02_sylvana extends OpMode {
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

    @Override
    public void init() {
        // Initialize drive motors
        motor_controller_drive = hardwareMap.dcMotorController.get("Motor_Controller_Drive");
        motor_drive_left = hardwareMap.dcMotor.get("Left_Motor");
        motor_drive_right = hardwareMap.dcMotor.get("Right_Motor");
        //capperMotor = hardwareMap.dcMotor.get("capperMotor")
        // TODO: 12/25/16 Declare
        //buttonServo = hardwareMap.servo.get("buttonServo");
        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        motor_drive_left.setDirection(DcMotor.Direction.REVERSE);
        //Declare positions of buttonServo // TODO: 12/25/16
        final double ARM_RETRACTED_POSITION = 0.2;
        final double ARM_EXTENDED_POSITION = 0.8;
    }

    @Override
    public void loop() {

        button_RT = gamepad1.right_trigger;
        button_LT = gamepad1.left_trigger;

        if (button_LT > 0) {
            if (button_LB) {
                motor_drive_left.setPower(-button_LT);
            } else {
                motor_drive_left.setPower(button_LT);
            }
        }
        if (button_RT > 0) {
            if (button_RB) {
                motor_drive_right.setPower(-button_RT);
            } else {
                motor_drive_right.setPower(button_RT);
            }
        }
        if (button_LT == 0 && button_RT == 0) {
            motor_drive_left.setPower(0);
            motor_drive_right.setPower(0);
        }
    }
}