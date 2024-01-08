package org.firstinspires.ftc.teamcode.CustomStuff;

import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class PIDtoPoint {

    PIDFController axialController;
    PIDFController lateralController;
    PIDFController angularController;


    MecanumDrive drive;

    public PIDtoPoint (double y, double x, double heading, MecanumDrive drive) {
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
        axialController.setSetPoint(y * Math.cos(drive.pose.heading.log()) - x * Math.sin(drive.pose.heading.log()));
        lateralController.setSetPoint(y * Math.sin(drive.pose.heading.log()) + x * Math.cos(drive.pose.heading.log()));
        angularController.setSetPoint(heading);
    }

    public void updateCoeffs(double[] axial, double[] lateral, double[] heading) {
        axialController.setPIDF(axial[0], axial[1], axial[2], drive.PARAMS.kS);
        lateralController.setPIDF(lateral[0], lateral[1], lateral[2], drive.PARAMS.kS);
        angularController.setPIDF(heading[0], heading[1], heading[2], drive.PARAMS.kS);
    }

    public boolean update() {
        if(axialController.atSetPoint() && lateralController.atSetPoint() && angularController.atSetPoint())
            return true;
        else {
            double xError = axialController.calculate(drive.pose.position.y * Math.cos(drive.pose.heading.log()) - drive.pose.position.x  * Math.sin(drive.pose.heading.log()));
            double yError = lateralController.calculate(drive.pose.position.y * Math.sin(drive.pose.heading.log()) + drive.pose.position.x * Math.cos(drive.pose.heading.log()));
            double headingError = angularController.calculate(drive.pose.heading.log());

            drive.leftFront.setPower(xError - yError - headingError);
            drive.leftBack.setPower(xError + yError - headingError);
            drive.rightBack.setPower(xError - yError + headingError);
            drive.rightFront.setPower(xError + yError + headingError);
            return false;
        }
    }
}


