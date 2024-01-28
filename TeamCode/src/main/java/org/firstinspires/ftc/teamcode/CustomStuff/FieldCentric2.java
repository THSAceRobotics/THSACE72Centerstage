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
    private GamepadEx clawOp;

    ArmState state = ArmState.PICK;

    private enum ArmState {
        PICK,
        MOVE,
        DROP
    }


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
        clawOp = new GamepadEx(gamepad2);

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

            if(clawOp.wasJustReleased(GamepadKeys.Button.DPAD_UP)) {
                if(state == ArmState.DROP) {
                    if(!(leftSlide.getCurrentPosition() >= 2700)) {
                        liftControl(leftSlide.getCurrentPosition() + 200);
                    }
                } else if (state == ArmState.MOVE) {
                    up = 0;
                    timer.reset();
                } else if (state == ArmState.PICK) {
                    claw.setPosition(.7);
                    spinArm(1);
                    liftControl(100);
                    state = ArmState.MOVE;
                }
            } else if (clawOp.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                if(state == ArmState.MOVE) {
                    liftControl(0);
                    state = ArmState.PICK;
                } else if (state == ArmState.DROP) {
                    if(leftSlide.getCurrentPosition() >= 1000) {
                        liftControl(leftSlide.getCurrentPosition() - 200);
                    }
                }
             } else if (clawOp.wasJustReleased(GamepadKeys.Button.DPAD_LEFT)) {
                if(state == ArmState.DROP) {
                    down = 0;
                    timer.reset();
                }
            } else if (clawOp.wasJustReleased((GamepadKeys.Button.DPAD_RIGHT))) {
                up = 5;
                down = 5;
                state = ArmState.DROP;
                liftControl(1500);
            }

            switch(up) {
                case 0: {
                    liftControl(1200);
                    if(timer.seconds() > 2)
                        up = 1;
                    break;
                }
                case 1: {
                    spinArm(.70);
                    state = ArmState.DROP;
                    up = 2;
                    break;
                }
            }



            if(clawOp.wasJustPressed(GamepadKeys.Button.B)) {
                claw.setPosition(0);
            }
            if(clawOp.wasJustPressed(GamepadKeys.Button.Y)) {
                spinArm(1);
            }
            if(clawOp.wasJustReleased(GamepadKeys.Button.X)) {
                liftControl(700);
            }
            switch(down) {
                case 0: {
                    spinArm(1);
                    if(timer.seconds() > .8)
                        down = 1;
                    break;
                }
                case 1: {
                    liftControl(100);
                    state = ArmState.MOVE;
                    down = 2;
                    break;
                }
            }

            driverOp.readButtons();
            clawOp.readButtons();
            isFieldCentric.readValue();
            leftSlide.setPower(slidePower);
            rightSlide.setPower(slidePower);
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
