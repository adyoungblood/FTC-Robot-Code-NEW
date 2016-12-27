/*
Modern Robotics ODS Wall Follow Example
Updated 11/4/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Optical Distance sensor named "ods"
Left drive train motor named "ml"  (two letters)
Right drive train motor named "mr"
Both motors need encoders

For more information, go to http://modernroboticsedu.com/course/view.php?id=5
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Wall_Follow", group = "MRI")
//@Disabled
public class Wall_Follow extends LinearOpMode {

    //Instance of OpticalDistanceSensor
    OpticalDistanceSensor ODS;

    //Motors
    DcMotor Left_Motor;
    DcMotor Right_Motor;

    //Raw value is between 0 and 1
    double odsReadingRaw;

    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;

    @Override
    public void runOpMode() throws InterruptedException {

        //identify the port of the ODS and motors in the configuration file
        ODS = hardwareMap.opticalDistanceSensor.get("ods1");
        Left_Motor = hardwareMap.dcMotor.get("Left_Motor");
        Right_Motor = hardwareMap.dcMotor.get("Right_Motor");

        //This program was designed around a robot that uses two gears on each side of the drive train.
        //If your robot uses direct drive between the motor and wheels or there are an odd number of gears, the opposite motor will need to be reversed.
        Right_Motor.setDirection(DcMotor.Direction.REVERSE);

        Left_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            odsReadingRaw = ODS.getRawLightDetected() / 5;                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear = Math.pow(odsReadingRaw, 0.5);                //calculate linear value

            //The below two equations operate the motors such that both motors have the same speed when the robot is the right distance from the wall
            //As the robot gets closer to the wall, the left motor received more power and the right motor received less power
            //The opposite happens as the robot moves further from the wall. This makes a proportional and elegant wall following robot.
            //See the video explanation on the Modern Robotics YouTube channel, the ODS product page, or modernroboticsedu.com.
            Left_Motor.setPower(odsReadingLinear * 2);
            Right_Motor.setPower(0.5 - (odsReadingLinear * 2));

            telemetry.addData("0 ODS Raw", odsReadingRaw);
            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 Motor Left", Left_Motor.getPower());
            telemetry.addData("3 Motor Right", Right_Motor.getPower());
            telemetry.update();
        }
    }
}//end class