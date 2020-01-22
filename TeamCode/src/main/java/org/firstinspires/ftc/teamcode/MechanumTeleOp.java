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

        Hardware robot = new Hardware();

        public void runOpMode() {

            robot.teleInit(hardwareMap);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            robot.runtime.reset();

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
                robot.frontLeft.setPower  (s * Range.clip(ly + rx + lx, -1.0, 1.0));
                robot.backLeft.setPower   (s * Range.clip(ly + rx - lx, -1.0, 1.0));
                robot.frontRight.setPower (s * Range.clip(ly - rx - lx, -1.0, 1.0));
                robot.backRight.setPower  (s * Range.clip(ly - rx + lx, -1.0, 1.0));

                //Tail controls
                if (gamepad1.dpad_down) {
                    robot.tail.setPosition(robot.TAIL_DOWN);
                }
                else if (gamepad1.dpad_up) {
                    robot.tail.setPosition(robot.TAIL_UP);
                }

                //Arm Controls
                double armPower = gamepad2.left_stick_y;
                if ((robot.ARM_BOTTOM_LIMIT < robot.arm.getCurrentPosition()) && (robot.arm.getCurrentPosition() < robot.ARM_TOP_LIMIT)) {
                    robot.arm.setPower(armPower*0.7);
                }
                else {
                    robot.arm.setPower(0);
                }

                //Elbow Controls
                if (gamepad2.right_stick_y > 0.1) {
                    robot.elbow.setPower(gamepad2.right_stick_y * 0.5);
                }
                else if (gamepad2.right_stick_y < 0.1) {
                    robot.elbow.setPower(gamepad2.right_stick_y *0.25);
                }
                else {
                    robot.elbow.setPower(0.0);
                }


                //Wrist Controls
                if (gamepad2.dpad_up) {
                        robot.wrist.setPosition(robot.wrist.getPosition() + 0.05);
                }
                else if (gamepad1.dpad_down) {
                        robot.wrist.setPosition(robot.wrist.getPosition() - 0.05);
                }
                else {
                    robot.wrist.setPosition(robot.wrist.getPosition());
                }

                //Finger Controls
                if (gamepad1.a) {
                    robot.finger.setPosition(robot.FINGER_CLOSED);
                }
                else if (gamepad1.y) {
                    robot.finger.setPosition(robot.FINGER_OPEN);
                }
                else if (gamepad1.b) {
                    robot.finger.setPosition(robot.FINGER_CAPSTONE);
                }

                /*double red = color.red();
                double blue = color.blue();
                double green = color.green();
                double alpha = color.alpha();

                telemetry.addData("Red:", red);
                telemetry.addData("Blue", blue);
                telemetry.addData("Green:", green);
                telemetry.addData("Alpha:", alpha); */

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
            }
        }
}
