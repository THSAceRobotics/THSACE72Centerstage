package org.firstinspires.ftc.teamcode.CustomStuff;

public class Profile {
    private State startState, endState;
    private double startVelo;
    private Constraints constraints;
    private double t1, t2, t3, totalTime;
    private double d1, d2, d3, totalDistance;
    private boolean tooShort;
    private double offset;

    static class State {
        private double accel;
        private double velo;
        private double pos;

        public State(double a, double v, double p) {
            accel = a;
            velo = v;
            pos = p;
        }

        public String toString() {
            return "Accel: " + accel + " Velo: " + velo + " Pos: " + pos;
        }
    }


    static class Constraints {
        private double maxAccel;
        private double maxDecel;
        private double maxVelo;


        public Constraints(double a, double v) {
            maxAccel = a;
            maxDecel = a;
            maxVelo = v;
        }

        public Constraints(double a, double d, double v) {
            maxAccel = a;
            maxDecel = d;
            maxVelo = v;
        }
    }

    public Profile(State s, State e, Constraints c) {
        startState = s;
        endState = e;
        constraints = c;
        totalDistance = Math.abs(startState.pos - endState.pos);
        if(endState.pos < startState.pos)
            startVelo = startState.velo * -1;
        else
            startVelo = startState.velo;
        t1 = (constraints.maxVelo - startVelo)/constraints.maxAccel;
        t3 = constraints.maxVelo/constraints.maxDecel;
        d1 = (constraints.maxAccel/2) * Math.pow(t1, 2) + t1 * startVelo;
        d3 = (constraints.maxDecel/2) * Math.pow(t3, 2);
        if(d3 + d1 > totalDistance) {
            d2 = 0;
            t2 = 0;
            tooShort = true;
        }
        else {
            d2 = totalDistance - (d1 + d3);
            t2 = d2/constraints.maxVelo;
        }
        offset = -(((2 * -(t1 + (constraints.maxAccel/constraints.maxDecel * t1) + t2)) * (-constraints.maxDecel / 2)) + startVelo) / -constraints.maxDecel;
        offset = totalDistance - ((-constraints.maxDecel/2) * Math.pow(offset - (t1 + (constraints.maxAccel/constraints.maxDecel * t1) + t2), 2) + startVelo * offset);
        totalTime = t1 + t2 + t3;
    }
    public State calculate(double time) {
        double p;
        double v;
        double a;
        if(tooShort){
            double accelDist;
            double decelDist;
            double cst = startVelo/constraints.maxDecel;
            cst = (constraints.maxDecel/2) * Math.pow(cst, 2);
            if(constraints.maxAccel > constraints.maxDecel) {
                double butt = constraints.maxAccel/constraints.maxDecel;
                accelDist = (totalDistance - cst) / (1 + butt);
                decelDist = totalDistance - accelDist;
            } else if(constraints.maxAccel < constraints.maxDecel) {
                double butt = constraints.maxDecel/constraints.maxAccel;
                decelDist = ((totalDistance - cst) / (1 + butt)) + cst;
                accelDist = totalDistance - decelDist;
            } else {
                accelDist = (totalDistance - cst) / 2;
                decelDist = totalDistance - accelDist;
            }
            double timeForAccel = (-startVelo + Math.sqrt(Math.pow(startVelo, 2) + -4 * (constraints.maxAccel/2) * -accelDist))/constraints.maxAccel;
            double timeForDecel = Math.sqrt(decelDist/(constraints.maxDecel/2));
            if(time > timeForDecel + timeForAccel)
                return new State(0, 0, endState.pos);
            if(time >= timeForAccel) {
                p = totalDistance - ((constraints.maxDecel/2) * Math.pow(time - (timeForAccel + timeForDecel), 2));
                v = constraints.maxAccel * timeForAccel + startVelo - constraints.maxDecel * (time - timeForAccel);
                a = -constraints.maxDecel;
            } else {
                p = (constraints.maxAccel/2) * Math.pow(time, 2) + time * startVelo;
                v = constraints.maxAccel * time + startVelo;
                a = constraints.maxAccel;
            }
        }
        else {
            if(time > totalTime)
                return new State(0, 0, endState.pos);
            if(time > t1 + t2) {
                p = -((constraints.maxDecel/2) * Math.pow(time - (t1 + (constraints.maxAccel/constraints.maxDecel * t1) + t2), 2)) + startVelo * time + offset;
                v = constraints.maxVelo - constraints.maxDecel * (time - t1 - t2);
                a = -constraints.maxDecel;
            }
            else if (time > t1) {
                p = d1 + constraints.maxVelo * (time - t1);
                v = constraints.maxVelo;
                a = 0;
            } else {
                p = (constraints.maxAccel/2) * Math.pow(time, 2) + time * startVelo;
                v = constraints.maxAccel * time + startVelo;
                a = constraints.maxAccel;
            }
        }
        if(endState.pos > startState.pos)
            return new State(a, v, p + startState.pos);
        else
            return new State(a, v, startState.pos - p);
    }
}
