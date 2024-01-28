package org.firstinspires.ftc.teamcode.CustomStuff;

import android.animation.ArgbEvaluator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "fieldctr")
@Config
public class FieldCentric extends LinearOpMode {

    private DcMotor fL, fR, bL, bR;
    private DcMotor leftSlide, rightSlide, intake;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private Servo claw;
    private Servo leftSpin, rightSpin;
    public static double servoPosLift = 0, servoPosDrop = 0, servoPosPick = 0;
    private IMU imu;


    @Override
    public void runOpMode() {
        /* instantiate motors */
        fL = hardwareMap.get(DcMotor.class,"leftFront");
        fR = hardwareMap.get(DcMotor.class,"rightFront");
        bL = hardwareMap.get(DcMotor.class,"leftBack");
        bR = hardwareMap.get(DcMotor.class,"rightBack");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
        intake = hardwareMap.get(DcMotor.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        leftSpin = hardwareMap.get(Servo.class, "leftSpin");
        rightSpin = hardwareMap.get(Servo.class,"rightSpin");

        driverOp = new GamepadEx(gamepad1);
        double slidePower = .7;
        imu.resetYaw();
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //might need to change
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD); //might need to change
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSpin.setDirection(Servo.Direction.REVERSE); // might need to change
        rightSpin.setDirection(Servo.Direction.FORWARD); // might need to change
        ToggleButtonReader isFieldCentric = new ToggleButtonReader(driverOp, GamepadKeys.Button.B);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();




        while (opModeIsActive()) {
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;


            if (isFieldCentric.getState()) {
                double rotX = x * Math.cos(-angle) - y * Math.sin(-angle);
                double rotY = x * Math.sin(-angle) + y * Math.cos(-angle);
                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                fL.setPower(frontLeftPower);
                bL.setPower(backLeftPower);
                fR.setPower(frontRightPower);
                bR.setPower(backRightPower);
            } else {
                x *= 1.1;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                fL.setPower(frontLeftPower * .7);
                bL.setPower(backLeftPower * .7);
                fR.setPower(frontRightPower * .7);
                bR.setPower(backRightPower * .7);
            }

            if(gamepad2.dpad_down) {
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (slidePower > 0) {
                        slidePower *= -1;
                    }
            } else if (gamepad2.dpad_up) {
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (slidePower < 0) {
                        slidePower *= -1;
                    }
            } else if (gamepad2.dpad_right){
                liftControl(leftSlide.getCurrentPosition());
            } else if (gamepad2.dpad_left) {
                liftControl(0);
            }

            if(gamepad1.right_bumper) {
                intake.setPower(1);
            } else if(gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }


            if (gamepad2.right_bumper) {
                claw.setPosition(1);
            } else if (gamepad2.left_bumper) {
                claw.setPosition(0);
            }

            if(gamepad2.a) {
                spinArm(servoPosPick);
            } else if (gamepad2.b) {
                spinArm(servoPosLift = .75);
            } else if (gamepad2.x) {
                spinArm(servoPosDrop = .45);
            }

            leftSlide.setPower(slidePower);
            rightSlide.setPower(slidePower);

            telemetry.addData("lift: ", servoPosLift);
            telemetry.addData("pick", servoPosPick);
            telemetry.addData("drop", servoPosDrop);
            telemetry.addData("rightslidepos: ", rightSlide.getCurrentPosition());
            telemetry.update();
        }
    }

    public void spinArm(double pos) {
        leftSpin.setPosition(pos);
        rightSpin.setPosition(pos);
    }

    private void liftControl(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(.7);
        rightSlide.setPower(.7);
    }

    public void up() {
        liftControl(100);
        spinArm(.95);
        liftControl(600);
        spinArm(.585);
    }
}
