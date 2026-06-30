package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class Turret_LM extends LinearOpMode {

    private static class PIDF {
        private double kP, kI, kD, kF;
        private double lastError = 0, integral = 0;
        private double feedForward = 0;

        public PIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public void updateError(double error) {
            integral += error;
            lastError = error;
        }

        public void updateFeedForwardInput(double ff) {
            feedForward = ff;
        }

        public double run() {
            return kP * lastError + kI * integral + kD * lastError + kF * feedForward;
        }

        public void reset() {
            lastError = 0;
            integral = 0;
            feedForward = 0;
        }
    }

    public static double rpt        = 0.00268785;
    public static double pidfSwitch = 60;
    public static double kp = 1.5, kf = 0.05, kd = 0.005;
    public static double sp = 0.5, sf = 0.03, sd = 0.001;

    // Confirmed via test footage: tx was climbing (1.39 -> 5.88) while target offset
    // was -5, meaning positive PIDF output drives the turret the WRONG way (increases
    // tx instead of decreasing it toward target). Flipped to -1.0 to correct this.
    public static double VISION_SIGN = -1.0;

    // Caps how fast the vision (fine-tracking) PIDF can drive the motor, since a sudden
    // tag jump (re-acquire, jitter) shouldn't snap the turret at full power.
    public static double VISION_MAX_POWER = 0.5;

    // Goal position on the field (inches)
    public static double GOAL_X = 0;
    public static double GOAL_Y = 144;

    private DcMotorEx turretMotor;
    private PIDF p, s;
    private double t = 0;
    private double error = 0;
    private boolean tracking = false;
    private boolean wasUsingVision = false;

    private Follower follower;
    // Set this start pose to match your exact starting position on the field tile layout
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(80);
        limelight.pipelineSwitch(0);
        limelight.start();

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setPower(0);

        p = new PIDF(kp, 0, kd, kf);
        s = new PIDF(sp, 0, sd, sf);

        seedTargetFromPose(startPose);
        tracking = true;

        telemetry.addData("Status", "Initialized & Tracking Active");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            LLResult result = limelight.getLatestResult();
            boolean usingVision = false;
            double targetOffsetDegrees = 0;
            int lockedTagId = -1;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : fiducialResults) {
                    int id = tag.getFiducialId();
                    if (id == 20) {
                        targetOffsetDegrees = -5.0;
                        usingVision = true;
                        lockedTagId = 20;
                        break;
                    } else if (id == 24) {
                        targetOffsetDegrees = 5.0;
                        usingVision = true;
                        lockedTagId = 24;
                        break;
                    }
                }
            }

            if (tracking) {
                double currentPosition = turretMotor.getCurrentPosition();
                double errorInRadians = 0;

                if (usingVision && result != null) {
                    // ---- FINE VISION TRACKING ----
                    // tx = camera-reported angle (deg) from centerline to tag.
                    // targetOffsetDegrees is the desired aim offset from "tag centered"
                    // (tag 20 -> -5 = aim 5° left of tag, tag 24 -> +5 = aim 5° right of tag).
                    // We're correctly aimed when tx == targetOffsetDegrees, so the error
                    // is how far tx still is from that target value.

                    if (!wasUsingVision) {
                        // Just switched into vision mode -- wipe stale PIDF state
                        // (lastError/integral/feedForward) so it doesn't carry over
                        // from odometry mode and cause a jerk/snap.
                        s.reset();
                    }

                    double tx = result.getTx();
                    double visionErrorDegrees = VISION_SIGN * (tx - targetOffsetDegrees);

                    // Small deadband so tiny sensor noise near zero doesn't keep
                    // triggering a feedforward kick and creating a slow drift.
                    double VISION_DEADBAND_DEG = 0.5;
                    if (Math.abs(visionErrorDegrees) < VISION_DEADBAND_DEG) {
                        visionErrorDegrees = 0;
                    }

                    errorInRadians = Math.toRadians(visionErrorDegrees);

                    error = errorInRadians / rpt;
                    t = currentPosition + error;

                    s.updateError(errorInRadians);
                    // Feedforward now scales with how far off we are (capped at 1),
                    // instead of always being a fixed +/-sf kick from signum(). This
                    // stops the constant small bias that was causing the slow creep.
                    double ffScale = Math.min(1.0, Math.abs(visionErrorDegrees) / 5.0);
                    s.updateFeedForwardInput(Math.signum(errorInRadians) * ffScale);

                    double visionPower = s.run();
                    visionPower = Math.max(-VISION_MAX_POWER, Math.min(VISION_MAX_POWER, visionPower));
                    turretMotor.setPower(visionPower);

                } else {
                    // ---- COARSE ODOMETRY TRACKING (FALLBACK) ----
                    if (wasUsingVision) {
                        // Just switched out of vision mode -- wipe stale PIDF state.
                        s.reset();
                        p.reset();
                    }

                    double angleToGoal    = Math.atan2(GOAL_Y - currentPose.getY(), GOAL_X - currentPose.getX());
                    double robotAngleDiff = normalizeAngle(angleToGoal - currentPose.getHeading());

                    // Constrain the travel bounds safety buffer
                    robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));

                    // Matches AutoTurretSub mapping (no sign inversion)
                    t     = robotAngleDiff / rpt;
                    error = t - currentPosition;
                    errorInRadians = error * rpt;

                    if (Math.abs(error) > pidfSwitch) {
                        p.updateError(errorInRadians);
                        p.updateFeedForwardInput(Math.signum(errorInRadians));
                        turretMotor.setPower(p.run());
                    } else {
                        s.updateError(errorInRadians);
                        s.updateFeedForwardInput(Math.signum(errorInRadians));
                        turretMotor.setPower(s.run());
                    }
                }
            } else {
                turretMotor.setPower(0);
            }

            wasUsingVision = usingVision;

            if (gamepad1.back || gamepad1.share) {
                tracking = false;
            }

            telemetry.addData("Tracking Mode", usingVision ? "VISION (Limelight)" : "ODOMETRY (Pedro)");
            if (usingVision && result != null) {
                telemetry.addData("Locked Tag ID", lockedTagId);
                telemetry.addData("tx (raw, deg)", "%.2f", result.getTx());
                telemetry.addData("Target Offset (deg)", targetOffsetDegrees);
                telemetry.addData("VISION_SIGN", VISION_SIGN);
            }
            telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Turret Target (t)", t);
            telemetry.addData("Turret Current Pos", turretMotor.getCurrentPosition());
            telemetry.addData("Turret Error (Ticks)", error);
            telemetry.update();
        }

        limelight.stop();
    }

    private void seedTargetFromPose(Pose pose) {
        double angleToGoal    = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
        double robotAngleDiff = normalizeAngle(angleToGoal - pose.getHeading());
        robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));
        t = robotAngleDiff / rpt;
    }

    private static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI)   angle -= Math.PI * 2D;
        return angle;
    }
}