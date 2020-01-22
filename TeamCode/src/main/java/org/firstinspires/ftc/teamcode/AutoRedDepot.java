package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto Red Depot")
public class AutoRedDepot extends LinearOpMode {

    Hardware robot = new Hardware();

    public void runOpMode() {

        robot.autoInit(hardwareMap);
        robot.initIMU();
        //robot.initVuforia();
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        waitForStart();

        //armOut();
        moveThatRobot(28,28,28, 28, 0.9, 500, 7000);
        //moveThatRobot(-2,2,2,-2,0.5,500);

        while (opModeIsActive() && robot.count <= 2) {
            if ((0 < robot.color.red()) && (robot.color.red()< 25)) {
                int red = robot.color.red();
                telemetry.addData("Red:", red);
                telemetry.update();
                //armDown();
                moveThatRobot(-4,-4,-4,-4,0.7,250, 2000);
                rotateRobot(-75,0.7);
                break;
            }
            else if ((25 < robot.color.red())) {
                robot.count += 1;
                int red = robot.color.red();
                telemetry.addData("Red:", red);
                telemetry.update();
                moveThatRobot(11, -11,-11,11,1,1000, 2500);
                moveThatRobot(3,3,3,3,0.5,500, 3000);
            }
        }

        moveThatRobot(72,72,72,72,0.8,1000, 10000);
        rotateRobot(75,0.7);
        rotateRobot(-165,0.7);
        moveThatRobot(6,6,6,6,0.5,1000, 5000);
        robot.tail.setPosition(robot.TAIL_DOWN);
        moveThatRobot(-24,-24,-24,-24,0.7,1000, 5000);

        //moveThatRobot(-24,-24,-24,-24,0.5,1000);
    }


    void rotateRobot(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        robot.resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.backLeft.setPower(leftPower);
        robot.frontLeft.setPower(leftPower);
        robot.backRight.setPower(rightPower);
        robot.frontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle() == 0) {}

            while (opModeIsActive() && robot.getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && robot.getAngle() < degrees) {}

        // turn the motors off.
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        robot.resetAngle();
    }

    public void moveThatRobot (double frontRightDistance, double frontLeftDistance, double backRightDistance, double backLeftDistance, double speed, long timeout, int max) {

        robot.runtime.reset();
        int newBackLeftTarget;
        int newFrontLeftTarget;
        int newBackRigtTarget;
        int newFrontRightTarget;

        if (opModeIsActive()) {
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int) (backLeftDistance * robot.WHEEL_CPI);
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int) (frontLeftDistance * robot.WHEEL_CPI);
            newBackRigtTarget = robot.backRight.getCurrentPosition() + (int) (backRightDistance * robot.WHEEL_CPI);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int) (frontRightDistance * robot.WHEEL_CPI);

            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.backRight.setTargetPosition(newBackRigtTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);

            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Turn on motors
            robot.backLeft.setPower(speed);
            robot.frontLeft.setPower(speed);
            robot.backRight.setPower(speed);
            robot.frontRight.setPower(speed);

            robot.maxtime.reset();
            while (opModeIsActive() && robot.backLeft.isBusy() && robot.frontLeft.isBusy() &&
                    robot.backRight.isBusy() && robot.frontRight.isBusy() && (robot.maxtime.milliseconds() < max)) {

            }

            //Stopping motors
            robot.backLeft.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontRight.setPower(0);

            //Turn off RUN_TO_POSITION
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(timeout);
        }

    }

    public void pickUp () {
        robot.arm.setTargetPosition((int)(robot.MOVE_ARM));
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.3);
        while (opModeIsActive() && robot.arm.isBusy()) {
        }
        robot.arm.setPower(0.0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.elbow.setTargetPosition((int)(robot.ELBOW_PICK));
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(0.8);
        while (opModeIsActive() && robot.elbow.isBusy()) {
        }
        robot.elbow.setPower(0.0);
        robot.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.wrist.setPosition(robot.WRIST_PICK);
        robot.runtime.reset();
        while (opModeIsActive() && (robot.runtime.seconds() < 1.5)) {
        }
        robot.finger.setPosition(robot.FINGER_OPEN);
        robot.runtime.reset();
        while (opModeIsActive() && (robot.runtime.seconds() < 1.5)) {
        }
    }

    public void armOut () {
        robot.arm.setTargetPosition((int)(robot.MOVE_ARM));
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.5);
        robot.runtime.reset();
        while (opModeIsActive() && robot.arm.isBusy() && (robot.runtime.seconds() < 4)) {
        }
        robot.arm.setPower(0.0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.elbow.setTargetPosition((int)(robot.ELBOW_OUT));
        telemetry.addData("Encoder:", robot.elbow.getCurrentPosition());
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(1);
        while (opModeIsActive() && robot.elbow.isBusy()) {
        }
        robot.elbow.setPower(0.0);
        robot.wrist.setPosition(robot.WRIST_PICK);
        robot.finger.setPosition(robot.FINGER_OPEN);
        robot.elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void armDown () {
        robot.elbow.setTargetPosition((int)(robot.ELBOW_PICK));
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elbow.setPower(0.4);
        while (opModeIsActive() && robot.elbow.isBusy()) {
        }
        robot.elbow.setPower(0.0);
    }
}
