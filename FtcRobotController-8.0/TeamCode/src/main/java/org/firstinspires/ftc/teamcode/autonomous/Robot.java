package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {
    public DcMotor frontLeft = null;
    public DcMotor rearLeft = null;
    public DcMotor frontRight = null;
    public DcMotor rearRight = null;
    public DcMotor liftMotor = null;

    HardwareMap hardwareMap;

    public Robot() {
    }

    public void init(HardwareMap hwMap) {

        hardwareMap = hwMap;

        //initialize motors and servos
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        //set direction of motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //set power to 0
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
        liftMotor.setPower(0);

        //set motors to use brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set to run using encoder mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    void setMotorPower(double power) {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
    }
    void setMotorMode(DcMotor.RunMode runmode) {
        frontLeft.setMode(runmode);
        rearLeft.setMode(runmode);
        frontRight.setMode(runmode);
        rearRight.setMode(runmode);
    }
    void callibrateRunToPosition() {
        if(frontLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION || rearLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION || frontRight.getMode() != DcMotor.RunMode.RUN_TO_POSITION || rearRight.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveSetTargetPosition(0,0,0,0);
        }
    }
    void driveSetTargetPosition(int frontLeftPosition, int rearLeftPosition, int frontRightPosition, int rearRightPosition) {
        frontLeft.setTargetPosition(frontLeftPosition);
        rearLeft.setTargetPosition(rearLeftPosition);
        frontRight.setTargetPosition(frontLeftPosition);
        rearRight.setTargetPosition(rearRightPosition);
    }
    void addToPosition(int frontLeftPosition, int rearLeftPosition, int frontRightPosition, int rearRightPosition) {
        frontLeft.setTargetPosition(frontLeft.getTargetPosition() + frontLeftPosition);
        rearLeft.setTargetPosition(rearLeft.getTargetPosition() + rearLeftPosition);
        frontRight.setTargetPosition(frontRight.getTargetPosition() + frontRightPosition);
        rearRight.setTargetPosition(rearRight.getTargetPosition() + rearRightPosition);
    }
    boolean motorsAreBusy() {
        return frontLeft.isBusy() && rearLeft.isBusy() && frontRight.isBusy() && rearRight.isBusy();
    }
}