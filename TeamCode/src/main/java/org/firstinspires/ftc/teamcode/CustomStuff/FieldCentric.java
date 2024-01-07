package org.firstinspires.ftc.teamcode.CustomStuff;

import android.animation.ArgbEvaluator;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
public class FieldCentric extends LinearOpMode {

    private DcMotor fL, fR, bL, bR;
    private DcMotor leftSlide, rightSlide;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private Servo claw;
    private Servo spin1, spin2;
    private IMU imu;


    @Override
    public void runOpMode() {
        /* instantiate motors */
        fL = hardwareMap.get(DcMotor.class,"LeftFront");
        fR = hardwareMap.get(DcMotor.class,"RightFront");
        bL = hardwareMap.get(DcMotor.class,"LeftBack");
        bR = hardwareMap.get(DcMotor.class,"RightBack");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        spin1 = hardwareMap.get(Servo.class, "spin1");
        spin2 = hardwareMap.get(Servo.class,"spin2");

        driverOp = new GamepadEx(gamepad1);
        boolean isFieldCentric = false;
        double slidePower = 1;
        imu.resetYaw();
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //might need to change
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD); //might need to change
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin1.setDirection(Servo.Direction.REVERSE); // might need to change
        spin2.setDirection(Servo.Direction.FORWARD); // might need to change

        waitForStart();

        liftControl(50);

        while (opModeIsActive()) {
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (!isFieldCentric) {
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
            } else {
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
            }

            if(gamepad2.dpad_down) {
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(slidePower > 0) {
                    slidePower *= -1;
                }
            } else if (gamepad2.dpad_up) {
                leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(slidePower < 0) {
                    slidePower *= -1;
                }
            } else {
                liftControl(leftSlide.getCurrentPosition());
            }

            if (gamepad2.x) {
                claw.setPosition(.45);
            } else if (gamepad2.y || !gamepad2.x) {
                claw.setPosition(0);
            }

            if(gamepad2.a) {
                //spinArm();
            } else if (gamepad2.b) {
                //spinArm();
            }

            if(gamepad1.right_bumper) {
                isFieldCentric = !isFieldCentric;
            }
            leftSlide.setPower(slidePower);
            rightSlide.setPower(slidePower);
        }
    }

    public void spinArm(double pos) {
        spin1.setPosition(pos);
        spin2.setPosition(pos);
    }

    public void liftControl(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
