package org.firstinspires.ftc.teamcode.CustomStuff;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "testvidion")
@Disabled
public class VisionTest extends LinearOpMode {

    private Motor fL, fR, bL, bR;
    private DcMotor slide;
    private MecanumDrive drive;
    private GamepadEx driverOp;
    private Servo servo;

    IMU imu;
    @Override
    public void runOpMode() {
        AprilTagProcessor apt = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal vp = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class,"Webcam 1"), apt);
        waitForStart();
        while (opModeIsActive()) {
            List<AprilTagDetection> b = apt.getFreshDetections();

            if(b != null && !b.isEmpty()) {
                telemetry.addData("X", b.get(0).ftcPose.x);
                telemetry.addData("y", b.get(0).ftcPose.y);
                telemetry.addData("bearing", b.get(0).ftcPose.bearing);
                telemetry.addData("range", b.get(0).ftcPose.range);
            }

            telemetry.update();
        }
    }
}
