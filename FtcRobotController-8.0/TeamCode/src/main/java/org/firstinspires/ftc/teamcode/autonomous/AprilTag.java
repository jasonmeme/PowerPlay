/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.ArrayList;

@Autonomous(name = "Autonomous", group = "Concept")
public class AprilTag extends LinearOpMode
{
    //movement
    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;
    private CRServo carouselServo = null; //continuos rotation servo to spin carousel
    private DcMotor intakeMotor = null;
    private DcMotor liftMotor = null;
    private Servo output = null;
    private BNO055IMU imu;
    Robot robot = new Robot();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .5, correction, rotation;
    private int turnError = 8;
    PIDController pidRotate, pidDrive;
    private static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per rotation for the GoBilda 5202 PLanetary Motor
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;


    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest.id == LEFT){
            move(1);
        }else if(tagOfInterest.id == MIDDLE){
            move(2);
        }else if(tagOfInterest.id == RIGHT){
            move(3);
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    //+1 = forward, -1 = backwards
    private void moveToPosition(double inches, double power, double direction) {
        double targetPosition = inches*COUNTS_PER_INCH;
        robot.callibrateRunToPosition();
        if (direction == 1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)targetPosition, (int)targetPosition, (int)targetPosition, (int)targetPosition);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    correction = pidDrive.performPID(getAngle());
                    robot.frontLeft.setPower(power-correction);
                    robot.rearLeft.setPower(power-correction);
                    robot.frontRight.setPower(power+correction);
                    robot.rearRight.setPower(power+correction);
                } else {
                    robot.setMotorPower(0);
                }
            }
        } else if (direction == -1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)-targetPosition, (int)-targetPosition, (int)-targetPosition, (int)-targetPosition);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    correction = pidDrive.performPID(getAngle());
                    robot.frontLeft.setPower(-power-correction);
                    robot.rearLeft.setPower(-power-correction);
                    robot.frontRight.setPower(-power+correction);
                    robot.rearRight.setPower(-power+correction);
                } else {
                    robot.setMotorPower(0);
                }
            }
        }
        correction = 0;
    }
    // + = left, - = right
    private void strafeToPosition(double inches, double power, double direction) {
        double targetPosition = inches * COUNTS_PER_INCH;
        robot.callibrateRunToPosition();

        if(direction == 1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)targetPosition*-1, (int)targetPosition, (int)targetPosition, (int)targetPosition *-1);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    robot.frontLeft.setPower(-power);
                    robot.rearLeft.setPower(power);
                    robot.frontRight.setPower(power);
                    robot.rearRight.setPower(-power);
                } else {
                    robot.setMotorPower(0);
                }
            }
        } else if (direction == -1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)targetPosition, (int)targetPosition*-1, (int)targetPosition*-1, (int)targetPosition);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    robot.frontLeft.setPower(power);
                    robot.rearLeft.setPower(-power);
                    robot.frontRight.setPower(-power);
                    robot.rearRight.setPower(power);
                } else {
                    robot.setMotorPower(0);
                }
            }
        }
    }
    private void turnToAngle(int degrees, double turnPower) {
        resetAngle();
        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();


        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.frontLeft.setPower(power);
                robot.rearLeft.setPower(power);
                robot.frontRight.setPower(-power);
                robot.rearRight.setPower(-power);
                sleep(100);
            }
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.frontLeft.setPower(-power);
                robot.rearLeft.setPower(-power);
                robot.frontRight.setPower(power);
                robot.rearRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.frontLeft.setPower(-power);
                robot.rearLeft.setPower(-power);
                robot.frontRight.setPower(power);
                robot.rearRight.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.setMotorPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void move(int placement){
        if (placement == 1){
            strafeToPosition(18, .3, 1);
            moveToPosition(25, .3, -1);
        }
        else if(placement == 2){
            strafeToPosition(18, .3, 1);
            moveToPosition(50, .3, -1);
            strafeToPosition(18, .3, -1);
        }
        else if(placement == 3){
            strafeToPosition(18, .3, -1);
            moveToPosition(25, .3, -1);
        }
    }

}