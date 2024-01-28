package org.firstinspires.ftc.teamcode.CustomStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CustomStuff.vision.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.MecanumDrive.FollowTrajectoryAction;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import com.acmerobotics.roadrunner.ftc.Actions;


import kotlin.sequences.Sequence;


class Lift implements Action {

    private DcMotor leftSlide, rightSlide;
    private int liftPos;

    public Lift (HardwareMap hw, int a) {
        leftSlide = hw.get(DcMotor.class, "leftSlide");
        rightSlide = hw.get(DcMotor.class,"rightSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //might need to change
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD); //might need to change
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPos = a;
    }

    @Override
    public boolean run(TelemetryPacket a) {
        liftControl(liftPos);
        return true;
    }
    private void liftControl(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
@Autonomous
@Config
public class AutoBlueLeft extends LinearOpMode {
    private DcMotor leftSlide, rightSlide, intake;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private Servo claw;
    private Servo spin1, spin2;
    private IMU imu;
    private VisionPortal vp;


    public static double spikeX = 24 + 9/Math.sqrt(2), spikeY = 28 + 9/Math.sqrt(2), spikeHeading = 225, backBoardx = 54, backBoardY = 28;
    public static int trajNum = 1;
    public static double intakepower = -.5;
    //private PropDetectionPipeline pdp = new PropDetectionPipeline();




    @Override
    public void runOpMode() {
        //pdp.isBlue = true;
        //vp = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pdp);
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
        intake = hardwareMap.get(DcMotor.class,"intake");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        spin1 = hardwareMap.get(Servo.class, "leftSpin");
        spin2 = hardwareMap.get(Servo.class,"rightSpin");
        Lift move = new Lift(hardwareMap, 50);
        Lift drop = new Lift(hardwareMap, 1200);
        imu.resetYaw();
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //might need to change
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD); //might need to change
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        spin1.setDirection(Servo.Direction.REVERSE); // might need to change
        spin2.setDirection(Servo.Direction.FORWARD); // might need to change
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        drive = new MecanumDrive(hardwareMap, new Pose2d(24-9, 72-9, Math.toRadians(270)));
        Pose2d startPose = new Pose2d(24-9, 72-9, Math.toRadians(270));
        Vector2d endVector = new Vector2d(51, 60);
        SequentialAction left = new SequentialAction(
                new InstantAction(() -> spinArm(1)),
                new InstantAction(() -> liftControl(200)),
                drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(29, 45), Math.toRadians(225)).build(),
                new InstantAction(() -> {
                    intake.setPower(-.5);
                    sleep(1000);
                }),
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(29, 45, Math.toRadians(225))).strafeToLinearHeading(new Vector2d(53.6, 44.5), Math.toRadians(180)).build(),
                        new SequentialAction(
                            new InstantAction(() -> liftControl(600)),
                            new InstantAction(() -> spinArm(.70))
                        )
                ),
                new InstantAction(() -> claw.setPosition(0)),
                new SleepAction(1),
                new InstantAction(() -> liftControl(1400)),
                new SleepAction(1),
                drive.actionBuilder(new Pose2d(53.6, 44.5, Math.toRadians(180))).lineToX(48).build(),
                drive.actionBuilder(new Pose2d(48, 45, Math.toRadians(180))).strafeToConstantHeading(endVector).build(),
                new InstantAction(() -> spinArm(1)),
                new SleepAction(2),
                new InstantAction(() -> liftControl(0)),
                new SleepAction(5)
        );

        SequentialAction middle = new SequentialAction(
                new InstantAction(() -> spinArm(1)),
                new InstantAction(() -> liftControl(200)),
                drive.actionBuilder(startPose).strafeToConstantHeading(new Vector2d(15, 37)).build(),
                new InstantAction(() -> {
                    intake.setPower(-.3);
                    sleep(1000);
                }),
                new ParallelAction(
                    drive.actionBuilder(new Pose2d(15, 37, Math.toRadians(270))).strafeToLinearHeading(new Vector2d(54, 37), Math.toRadians(180)).build(),
                        new SequentialAction(
                                new InstantAction(() -> liftControl(600)),
                                new InstantAction(() -> spinArm(.70))
                        )
                ),
                new InstantAction(() -> claw.setPosition(0)),
                new SleepAction(1),
                new InstantAction(() -> liftControl(1400)),
                new SleepAction(1),
                drive.actionBuilder(new Pose2d(54, 37, Math.toRadians(180))).lineToX(48).build(),
                drive.actionBuilder(new Pose2d(48, 34, Math.toRadians(180))).strafeToConstantHeading(endVector).build(),
                new InstantAction(() -> spinArm(1)),
                new SleepAction(2),
                new InstantAction(() -> liftControl(0)),
                new SleepAction(5)
        );
        SequentialAction right = new SequentialAction(
                new InstantAction(() -> spinArm(1)),
                new InstantAction(() -> liftControl(200)),
                drive.actionBuilder(startPose).lineToY(34).build(),
                drive.actionBuilder(new Pose2d(startPose.position.x, 34, Math.toRadians(270))).strafeToLinearHeading(new Vector2d(6,  38.36396103067893), Math.toRadians(225)).build(),
                new InstantAction(() -> {
                    intake.setPower(-.3);
                    sleep(1000);
                }),
                new ParallelAction(
                    drive.actionBuilder(new Pose2d(6,  38.36396103067893, Math.toRadians(225))).strafeToLinearHeading(new Vector2d(54, 30), Math.toRadians(180)).build(),
                        new SequentialAction(
                                new InstantAction(() -> liftControl(800)),
                                new InstantAction(() -> spinArm(.70))
                        )
                ),
                new InstantAction(() -> claw.setPosition(0)),
                new SleepAction(1),
                new InstantAction(() -> liftControl(1400)),
                new SleepAction(1),
                drive.actionBuilder(new Pose2d(54, 30, Math.toRadians(180))).lineToX(48).build(),
                drive.actionBuilder(new Pose2d(48, 28, Math.toRadians(180))).strafeToConstantHeading(endVector).build(),
                new InstantAction(() -> spinArm(1)),
                new SleepAction(2),
                new InstantAction(() -> liftControl(0)),
                new SleepAction(5)
        );

        //vp.resumeStreaming();
        int location = trajNum;
        while(opModeInInit()) {
            //location = pdp.location;
        }

        waitForStart();
        //vp.stopStreaming();
        if(location == 1) {
            Actions.runBlocking(left);
        } else if (location == 2) {
            Actions.runBlocking(middle);
        } else if (location == 3){
            Actions.runBlocking(right);
        }


    }

    private void liftControl(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(1);
        rightSlide.setPower(1);
    }

    private Action balls(int pos) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftSlide.setTargetPosition(pos);
                rightSlide.setTargetPosition(pos);
                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlide.setPower(1);
                rightSlide.setPower(1);
                return true;
            }
        };
    }



    public void spinArm(double pos) {
        spin1.setPosition(pos);
        spin2.setPosition(pos);
    }
}
