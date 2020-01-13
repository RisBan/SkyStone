package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Mecanum OpMode", group="Linear Opmode")
public class MechanumTeleOp extends LinearOpMode{

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        DcMotor frontRight = null;
        DcMotor frontLeft = null;
        DcMotor backLeft = null;
        DcMotor backRight = null;
        Servo tail = null;
        //Servo claw = null;
        ColorSensor color;



        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight  = hardwareMap.get(DcMotor.class, "backRight");
            frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
            tail = hardwareMap.get(Servo.class, "tail");
            color = hardwareMap.get(ColorSensor.class, "color");
            //claw = hardwareMap.get(Servo.class, "claw");



            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);



            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                double lx = gamepad1.left_stick_x;
                double ly = -gamepad1.left_stick_y;

                double rx = gamepad1.right_stick_x;
                double ry = -gamepad1.right_stick_y;


                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;
            /*
            fl = ly + rx + lx
            bl = ly + rx - lx
            fr = ly - rx - lx
            br = ly - rx + lx
             */

                double s = .8;

                // Send calculated power to wheels
                frontLeft.setPower  (s * Range.clip(ly + rx + lx, -1.0, 1.0));
                backLeft.setPower   (s * Range.clip(ly + rx - lx, -1.0, 1.0));
                frontRight.setPower (s * Range.clip(ly - rx - lx, -1.0, 1.0));
                backRight.setPower  (s * Range.clip(ly - rx + lx, -1.0, 1.0));

                if (gamepad1.dpad_down) {
                    tail.setPosition(0.7);
                }
                else if (gamepad1.dpad_up) {
                    tail.setPosition(0.2);
                }

                /*if (gamepad1.y) {
                    claw.setPosition(0.9);
                }
                else if (gamepad1.a) {
                    claw.setPosition(0.0);
                }*/

                double red = color.red();
                double blue = color.blue();
                double green = color.green();
                double alpha = color.alpha();

                telemetry.addData("Red:", red);
                telemetry.addData("Blue", blue);
                telemetry.addData("Green:", green);
                telemetry.addData("Alpha:", alpha);

/*                if (gamepad1.y) {
                    frontRight.setPower(0.5);
                    frontLeft.setPower(0.0);
                    backRight.setPower(0.0);
                    backLeft.setPower(0.0);
                }
                else if (gamepad1.b) {
                    frontRight.setPower(0.0);
                    frontLeft.setPower(0.5);
                    backRight.setPower(0.0);
                    backLeft.setPower(0.0);
                }
                else if (gamepad1.a) {
                    frontRight.setPower(0.0);
                    frontLeft.setPower(0.0);
                    backRight.setPower(0.5);
                    backLeft.setPower(0.0);
                }
                else if (gamepad1.x) {
                    frontRight.setPower(0.0);
                    frontLeft.setPower(0.0);
                    backRight.setPower(0.0);
                    backLeft.setPower(0.5);
                }*/



                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Motors", "left (%.2f), right (%.2f)", motorPower, rightPower);
                telemetry.update();
            }
        }
}
