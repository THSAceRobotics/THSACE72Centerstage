package org.firstinspires.ftc.teamcode.CustomStuff;

import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CustomStuff.vision.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class AutoBlueRight extends LinearOpMode {
    private DcMotor leftSlide, rightSlide;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private Servo claw;
    private Servo spin1, spin2;
    private IMU imu;
    private VisionPortal vp;
    private PropDetectionPipeline pdp = new PropDetectionPipeline();



    @Override
    public void runOpMode() {
        vp = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pdp);
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
        claw = hardwareMap.get(Servo.class, "claw");
        imu = hardwareMap.get(IMU.class, "imu");
        spin1 = hardwareMap.get(Servo.class, "spin1");
        spin2 = hardwareMap.get(Servo.class,"spin2");
        imu.resetYaw();
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE); //might need to change
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD); //might need to change
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin1.setDirection(Servo.Direction.REVERSE); // might need to change
        spin2.setDirection(Servo.Direction.FORWARD); // might need to change


        drive = new MecanumDrive(hardwareMap, new Pose2d(24-9, 72-9, Math.PI));
        Pose2d startPose = new Pose2d(-33, 72-9, Math.PI);
        Vector2d endVector = new Vector2d(51, 60);
        SequentialAction left = new SequentialAction(
                drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(-48+9/Math.sqrt(2), 34+(9/Math.sqrt(2))), Math.toRadians(225)).build(),
                new InstantAction(() -> liftControl(50)),
                drive.actionBuilder(drive.pose).setTangent(Math.PI+1.3613902).splineToLinearHeading(new Pose2d(-16, 6, Math.toRadians(180)), Math.toRadians(0)).strafeToConstantHeading(new Vector2d(20, 6)).setTangent(Math.toRadians(0)).splineToLinearHeading(new Pose2d(51, 28, Math.toRadians(180)), Math.toRadians(90)).build(),
                new InstantAction(() -> liftControl(50)),
                drive.actionBuilder(drive.pose).strafeToConstantHeading(endVector).build()
        );

        SequentialAction middle = new SequentialAction(
                drive.actionBuilder(startPose).strafeToConstantHeading(new Vector2d(15, 33)).build(),
                new InstantAction(() -> liftControl(50)),
                drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(51, 34), Math.toRadians(180)).build(),
                new InstantAction(() -> liftControl(50)),
                drive.actionBuilder(drive.pose).strafeToConstantHeading(endVector).build()
        );
        SequentialAction right = new SequentialAction(
                drive.actionBuilder(startPose).strafeToLinearHeading(new Vector2d(24 + 9/Math.sqrt(2),  28 + 9/Math.sqrt(2)), Math.toRadians(225)).build(),
                new InstantAction(() -> liftControl(50)),
                drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(51, 28), Math.toRadians(180)).build(),
                new InstantAction(() -> liftControl(50)),
                drive.actionBuilder(drive.pose).strafeToConstantHeading(endVector).build()
        );

        vp.resumeStreaming();
        int location = 3;
        while(opModeInInit()) {
            location = pdp.location;
        }

        waitForStart();
        vp.stopStreaming();
        /*
        if(location == 1) {
            Actions.runBlocking(left);
        } else if (location == 2) {
            Actions.runBlocking(middle);
        } else {
            Actions.runBlocking(right);
        } */
    }

    public void liftControl(int pos) {
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void spinArm(double pos) {
        spin1.setPosition(pos);
        spin2.setPosition(pos);
    }
}
