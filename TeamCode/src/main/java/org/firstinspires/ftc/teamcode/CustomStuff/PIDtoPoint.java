package org.firstinspires.ftc.teamcode.CustomStuff;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class PIDtoPoint {

    PIDFController axialController;
    PIDFController lateralController;
    PIDFController angularController;
    DriveConstants dc = new DriveConstants();


    SampleMecanumDrive drive;

    public PIDtoPoint (double y, double x, double heading, SampleMecanumDrive drive) {
        axialController = new PIDFController(0,0,0,0);
        lateralController = new PIDFController(0,0,0,0);
        angularController = new PIDFController(0,0,0,0);
        axialController.setTolerance(1);
        lateralController.setTolerance(1);
        angularController.setTolerance(1);
        axialController.setSetPoint(y);
        lateralController.setSetPoint(x);
        angularController.setSetPoint(heading);
        this.drive = drive;
    }

    public void updatePIDSetPoints (double y, double x, double heading) {
        axialController.setSetPoint(y * Math.cos(drive.getPoseEstimate().getHeading()) - x * Math.sin(drive.getPoseEstimate().getHeading()));
        lateralController.setSetPoint(y * Math.sin(drive.getPoseEstimate().getHeading()) + x * Math.cos(drive.getPoseEstimate().getHeading()));
        angularController.setSetPoint(heading);
    }

    public void updateCoeffs(double[] axial, double[] lateral, double[] heading) {
        axialController.setPIDF(axial[0], axial[1], axial[2], dc.kStatic);
        lateralController.setPIDF(lateral[0], lateral[1], lateral[2], dc.kStatic);
        angularController.setPIDF(heading[0], heading[1], heading[2], dc.kStatic);
    }

    public boolean update() {
        if(axialController.atSetPoint() && lateralController.atSetPoint() && angularController.atSetPoint())
            return true;
        else {
            double xError = axialController.calculate(drive.getPoseEstimate().getY() * Math.cos(drive.getPoseEstimate().getHeading()) - drive.getPoseEstimate().getX()  * Math.sin(drive.getPoseEstimate().getHeading()));
            double yError = lateralController.calculate(drive.getPoseEstimate().getY() * Math.sin(drive.getPoseEstimate().getHeading()) + drive.getPoseEstimate().getX() * Math.cos(drive.getPoseEstimate().getHeading()));
            double headingError = angularController.calculate(drive.getPoseEstimate().getHeading());

            drive.setMotorPowers(xError - yError - headingError, xError + yError - headingError, xError - yError + headingError, xError + yError + headingError);

            return false;
        }
    }
}


