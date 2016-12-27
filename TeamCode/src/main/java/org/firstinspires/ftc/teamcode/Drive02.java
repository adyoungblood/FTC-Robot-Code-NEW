package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(name = "Drive02", group = "Tutorials")
class Drive02 extends LinearOpMode
{
    // Initialization of joystick buttons:
    public double button_RT;
    public double button_LT;

    public boolean button_RB;
    public boolean button_LB;
    // Declare drive motors
    private DcMotor motorLeft;
    private DcMotor motorRight;
    //private DcMotor capperMotor;
    //Declare servo motor // TODO: 12/25/16
    //private Servo buttonServo;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        //capperMotor = hardwareMap.dcMotor.get("capperMotor")
        // TODO: 12/25/16 Declare
        //buttonServo = hardwareMap.servo.get("buttonServo");
        // If drive motors are given full power, robot would spin because of the motors being in
        // opposite directions. So reverse one
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //Declare positions of buttonServo // TODO: 12/25/16
        //private static final double ARM_RETRACTED_POSITION = 0.2;
        //private static final double ARM_EXTENDED_POSITION = 0.8;

        // Wait until start button is pressed
        waitForStart();

        // Repeatedly run code in here until stop button is pressed
        while(opModeIsActive())
        {
            //Push button // TODO: 12/25/16
                /*if (gamepad.x)
                {
                    buttonServo.setPosition(ARM_EXTENDED_POSITION);
                */
            // Move drivetrain if requested
            if (0 < button_RT && 0 < button_LT)
            {
                motorLeft.setPower(button_LT);
                // Both set to same button to prevent the veering of motors; and to ensure robot actually goes straight
                motorRight.setPower(button_LT);
            }
            if(button_RB && button_LB)
            {
                // Both set to same button to prevent the veering of motors; and to ensure robot actually goes straight
                motorLeft.setPower(-1);
                motorRight.setPower(-1);
            }
            if(button_RB && button_LB)
            {
                // change done to previous ones not done to this, due to the fact that steering needs max. manueverability
                motorLeft.setPower(1);
                motorRight.setPower(-1);
            }
            if(button_RB && 0 < button_LT)
            {
                motorLeft.setPower(-1);
                motorRight.setPower(1);
            }
            else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
                //arm resets to default // TODO: 12/25/16
                //pushButton.setPosition(ARM_RETRACTED_POSITION);
            }
            // Give hardware a chance to catch up
            idle();
        }
    }
}