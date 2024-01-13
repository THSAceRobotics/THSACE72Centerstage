package org.firstinspires.ftc.teamcode.CustomStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "fieldctr2")
public class FieldCentric2 extends LinearOpMode {

    private DcMotor fL, fR, bL, bR;
    private DcMotor leftSlide, rightSlide, intake;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private Servo claw;
    public static double yes = .84;
    private Servo leftSpin, rightSpin;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        driverOp = new GamepadEx(gamepad1);
        double slidePower = .7;
        imu.resetYaw();
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //might need to change
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD); //might need to change
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSpin.setDirection(Servo.Direction.REVERSE); // might need to change
        rightSpin.setDirection(Servo.Direction.FORWARD); // might need to change
        ElapsedTime timer = new ElapsedTime();
        ToggleButtonReader isFieldCentric = new ToggleButtonReader(driverOp, GamepadKeys.Button.B);

        waitForStart();

        timer.startTime();
        int down = 5, up = 5;
        while (opModeIsActive()) {
            double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if (isFieldCentric.getState()) {
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
                if(leftSlide.getCurrentPosition() > 1200) {
                    leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (slidePower > 0) {
                        slidePower *= -1;
                    }
                } else {
                    liftControl(leftSlide.getCurrentPosition());
                }
            } else if (gamepad2.dpad_up) {
                if(leftSlide.getCurrentPosition() < 2700) {
                    if (leftSlide.getCurrentPosition() > 350) {
                        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        if (slidePower < 0) {
                            slidePower *= -1;
                        }
                    } else {
                        timer.reset();
                        up = 1;
                    }
                } else {
                    liftControl(leftSlide.getCurrentPosition());
                }
            } else if (gamepad2.dpad_right){
                liftControl(leftSlide.getCurrentPosition());
            } else if (gamepad2.dpad_left) {
                timer.reset();
                down = 1;
            }


            switch(up) {
                case 1: {
                    spinArm(.89);
                    claw.setPosition(.7);
                    if (timer.seconds() > 1)
                        up = 2;
                    break;
                }
                case 2: {
                    liftControl(1400);
                    if (leftSlide.getCurrentPosition() > 1000) {
                        up = 3;
                    }
                    break;
                }
                case 3: {
                    spinArm(.585);
                    up = 4;
                    break;
                }
            }

            switch(down) {
                case 1: {
                    spinArm(.89);
                    if(timer.seconds() > 1)
                        down = 2;
                    break;
                }
                case 2: {
                    liftControl(150);
                    if (leftSlide.getCurrentPosition() < 155) {
                        down = 3;
                    }
                    break;
                }
                case 3: {
                    spinArm(.87);
                    down = 4;
                    timer.reset();
                    break;
                }
                case 4:
                    if(timer.seconds() > .5) {
                        liftControl(0);
                        down = 5;
                        timer.reset();
                    }
                    break;
                case 5:
                    if(timer.seconds() > .5) {
                        timer.reset();
                        down = 6;
                    }
                    break;
            }

            if(leftSlide.getCurrentPosition() > 2700  && slidePower > 0)
                liftControl(leftSlide.getCurrentPosition());

            if (leftSlide.getCurrentPosition() < 1300 && up > 3 && slidePower < 0 && down > 5)
                liftControl(leftSlide.getCurrentPosition());


            if(gamepad1.right_bumper) {
                intake.setPower(1);
            } else if(gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }


            if (gamepad2.y) {
                claw.setPosition(0);
            }


            leftSlide.setPower(slidePower);
            rightSlide.setPower(slidePower);

            telemetry.addData("up: ", up);
            telemetry.addData("spos", leftSlide.getCurrentPosition());
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
    }


}
