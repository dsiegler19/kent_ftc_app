package edu.usrobotics.opmode.compbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import edu.usrobotics.opmode.LoggedOp;
import edu.usrobotics.opmode.RobotOp;
import edu.usrobotics.opmode.Route;
import edu.usrobotics.opmode.task.ConcurrentTaskSet;
import edu.usrobotics.opmode.task.Goal;
import edu.usrobotics.opmode.task.MotorTask;
import edu.usrobotics.opmode.task.Task;
import edu.usrobotics.opmode.task.TaskType;

/**
 * Created by dsiegler19 on 10/13/16.
 */
public abstract class CompbotAuto extends RobotOp {

    CompbotHardware robot = new CompbotHardware();
    private final boolean isBlueTeam;

    public CompbotAuto (boolean isBlueTeam) {
        this.isBlueTeam = isBlueTeam;
    }

    float buttonPressingDistance = 5f;
    float initialButtonPressingDistance = 15f;
    Goal<Integer> encoderGoal7 = new Goal<> (robot.inchesToEncoderTicks(buttonPressingDistance));
    Goal<Integer> encoderGoal7R = new Goal<> (robot.inchesToEncoderTicks(buttonPressingDistance));
    Goal<Integer> encoderGoal72 = new Goal<> (robot.inchesToEncoderTicks((buttonPressingDistance * 1.5f) + 5f));
    Goal<Integer> encoderGoal72R = new Goal<> (robot.inchesToEncoderTicks((buttonPressingDistance * 1.5f) + 5f));
    Goal<Integer> encoderGoal8 = new Goal<> (robot.inchesToEncoderTicks(buttonPressingDistance - 0.5f));
    Goal<Integer> encoderGoal8R = new Goal<> (robot.inchesToEncoderTicks(buttonPressingDistance - 0.5f));

    boolean onSecondBeacon = false;
    int numTimesHitBeacon = 1;
    boolean hitTheBeaconOnce = false;

    Task checkIfCorrectColor = null;

    @Override
    public void init() {

        robot.init(hardwareMap);

        final Route happyTrail = new Route();

        robot.setDirection(CompbotHardware.MovementDirection.NORTH);

        final int maxMotorSpeed = 50000;

        Task shoot = new Task() {
            ElapsedTime shooterTime;
            boolean completed = false;

            @Override
            public boolean execute() {
                if (shooterTime.seconds() > 2)
                    return true;


                if(isBlueTeam){

                    robot.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.shooterRight.setPower(1);
                    robot.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.shooterLeft.setPower(0.85);

                }

                else{

                    robot.shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.shooterRight.setPower(0.85);
                    robot.shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.shooterLeft.setPower(1);


                }

                robot.harvester.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.harvester.setPower(0.5f);

                return false;
            }

            @Override
            public boolean isCompleted() {
                return completed;
            }

            @Override
            public TaskType getType() {
                return TaskType.MOTOR;
            }

            @Override
            public void onReached() {
                shooterTime = new ElapsedTime();
            }

            @Override
            public boolean onExecuted() {
                return false;
            }

            @Override
            public void onCompleted() {
                robot.shooterRight.setPower(0);
                robot.shooterLeft.setPower(0);
                robot.harvester.setPower(0);

                completed = true;
            }

        };

        // Forward then turn the forward as an alternative to diagonals
        Goal<Integer> forward1EncoderGoal = new Goal<> (robot.inchesToEncoderTicks(6f));
        ConcurrentTaskSet forward = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, forward1EncoderGoal, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, forward1EncoderGoal, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, forward1EncoderGoal, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, forward1EncoderGoal, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted(0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.NORTH);

            }
        };

        // Diagonal to the beacon wall
        Goal<Integer> encoderGoal3 = new Goal<> (robot.inchesStraifingToEncoderTicks(50f));
        ConcurrentTaskSet crabToLine1Blue = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal3, maxMotorSpeed, 0.6f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal3, maxMotorSpeed, 0.6f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal3, maxMotorSpeed, 0f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal3, maxMotorSpeed, 0f, 0.7f, null, 0.1f)

        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted(0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.NORTH_EAST);

            }

        };

        Goal<Integer> encoderGoal3AndAHalf = new Goal<> (robot.inchesStraifingToEncoderTicks(100f));
        ConcurrentTaskSet crabToLine2Blue = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal3AndAHalf, maxMotorSpeed, 0.4f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal3AndAHalf, maxMotorSpeed, 0.4f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal3AndAHalf, maxMotorSpeed, 0f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal3AndAHalf, maxMotorSpeed, 0f, 0.7f, null, 0.1f)

        ) {
            @Override
            public boolean onExecuted() {
                return robot.sensingWhite(robot.bottomRightColorSensor) || robot.sensingWhite(robot.bottomLeftColorSensor);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.NORTH_EAST);

            }

        };

        // Diagonal to the beacon wall
        Goal<Integer> encoderGoal3R = new Goal<> (robot.inchesStraifingToEncoderTicks(50f));
        ConcurrentTaskSet crabToLine1Red = new ConcurrentTaskSet(
                new MotorTask(robot.frontRight, encoderGoal3R, maxMotorSpeed, 0.6f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal3R, maxMotorSpeed, 0.6f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontLeft, encoderGoal3R, maxMotorSpeed, 0f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal3R, maxMotorSpeed, 0f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted(0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.NORTH_WEST);

            }

        };

        Goal<Integer> encoderGoal3AndAHalfR = new Goal<> (robot.inchesStraifingToEncoderTicks(100f));
        ConcurrentTaskSet crabToLine2Red = new ConcurrentTaskSet(
                new MotorTask(robot.frontRight, encoderGoal3AndAHalfR, maxMotorSpeed, 0.4f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal3AndAHalfR, maxMotorSpeed, 0.4f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontLeft, encoderGoal3AndAHalfR, maxMotorSpeed, 0f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal3AndAHalfR, maxMotorSpeed, 0f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return robot.sensingWhite(robot.bottomRightColorSensor) || robot.sensingWhite(robot.bottomLeftColorSensor);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.NORTH);

            }

        };

        Goal<Integer> encoderGoal3AndThreeQuarters = new Goal<> (robot.inchesToEncoderTicks(4.5f));
        final ConcurrentTaskSet calibrationBackUp = new ConcurrentTaskSet(
                new MotorTask(robot.frontRight, encoderGoal3AndThreeQuarters, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontLeft, encoderGoal3AndThreeQuarters, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal3AndThreeQuarters, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal3AndThreeQuarters, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f)
        ) {

            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1) || robot.sensingWhite(robot.bottomRightColorSensor) || robot.sensingWhite(robot.bottomLeftColorSensor);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.SOUTH);

            }
        };

        Goal<Integer> encoderGoal3And78 = new Goal<> (robot.inchesToEncoderTicks(3f));
        final ConcurrentTaskSet calibrationBackUpSecondBeacon = new ConcurrentTaskSet(
                new MotorTask(robot.frontRight, encoderGoal3And78, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontLeft, encoderGoal3And78, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal3And78, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal3And78, maxMotorSpeed, 0.3f, 0.7f, null, 0.1f)
        ) {

            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.SOUTH);

            }
        };

        final ConcurrentTaskSet backUpBlue = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal8, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal8, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal8, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal8, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f)
        ) {

            @Override
            public boolean onExecuted() {

                return isTaskCompleted (0) || isTaskCompleted(1);

            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.WEST);

            }

        };

        final ConcurrentTaskSet backUpRed = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal8R, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal8R, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal8R, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal8R, maxMotorSpeed, 0.2f, 0.7f, null, 0.1f)
        ) {

            @Override
            public boolean onExecuted() {

                return isTaskCompleted (0) || isTaskCompleted(1);

            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.EAST);

            }

        };

        Goal<Integer> encoderGoalCW = new Goal<>(robot.inchesToEncoderTicks((initialButtonPressingDistance / 2) + 1f));
        final ConcurrentTaskSet beaconHitTwistClockwise = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoalCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoalCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoalCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoalCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onCompleted(){

                happyTrail.addTask(isBlueTeam ? backUpBlue : backUpRed);
                happyTrail.addTask(checkIfCorrectColor);

            }

            @Override
            public void onReached() {
                super.onReached();

                if(isBlueTeam){

                    robot.setDirection(CompbotHardware.MovementDirection.EAST);

                }

                else{

                    robot.setDirection(CompbotHardware.MovementDirection.WEST);

                }

            }

        };

        Goal<Integer> encoderGoalCCW = new Goal<>(robot.inchesToEncoderTicks((initialButtonPressingDistance / 2) + 1f));
        final ConcurrentTaskSet beaconHitTwistCounterclockwise = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoalCCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoalCCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoalCCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoalCCW, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onCompleted(){

                happyTrail.addTask(isBlueTeam ? backUpBlue : backUpRed);
                happyTrail.addTask(checkIfCorrectColor);

            }

            @Override
            public void onReached() {
                super.onReached();

                if(isBlueTeam){

                    robot.setDirection(CompbotHardware.MovementDirection.EAST);

                }

                else{

                    robot.setDirection(CompbotHardware.MovementDirection.WEST);

                }

            }

        };

        // Make the first approach to the beacon
        Goal<Integer> encoderGoal6 = new Goal<> (robot.inchesToEncoderTicks(initialButtonPressingDistance));
        final ConcurrentTaskSet beaconFirstApproachBlue = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal6, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal6, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal6, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal6, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {

                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onCompleted(){

                happyTrail.addTask(isBlueTeam ? backUpBlue : backUpRed);
                happyTrail.addTask(checkIfCorrectColor);

            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.EAST);

            }

        };

        // Make the first hit to the beacon
        Goal<Integer> encoderGoal6R = new Goal<> (robot.inchesToEncoderTicks(initialButtonPressingDistance));
        ConcurrentTaskSet beaconFirstApproachRed = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal6R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal6R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal6R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal6R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.WEST);

            }

            @Override
            public void onCompleted(){

                happyTrail.addTask(isBlueTeam ? backUpBlue : backUpRed);
                happyTrail.addTask(checkIfCorrectColor);

            }

        };

        // Make the next few hits to the beacon
        final ConcurrentTaskSet beaconHitBlue = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal7, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal7, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal7, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal7, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.EAST);

            }

        };

        final ConcurrentTaskSet beaconHitRed = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal7R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal7R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal7R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal7R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.WEST);

            }

        };

        // Make the next few hits to the beacon
        final ConcurrentTaskSet beaconHitBlue2 = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal72, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal72, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal72, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal72, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.EAST);

            }

        };

        final ConcurrentTaskSet beaconHitRed2 = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal72R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal72R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal72R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal72R, maxMotorSpeed, 0.5f, 0.7f, null, 0.1f)
        ) {
            @Override
            public boolean onExecuted() {
                return isTaskCompleted (0) || isTaskCompleted(1);
            }

            @Override
            public void onReached() {
                super.onReached();

                robot.setDirection(CompbotHardware.MovementDirection.WEST);

            }

        };

        final Goal<Integer> encoderGoal8AndAHalf = new Goal<>(20);
        final ConcurrentTaskSet align = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal8AndAHalf, maxMotorSpeed, 0.55f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal8AndAHalf, maxMotorSpeed, 0.55f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal8AndAHalf, maxMotorSpeed, 0.55f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal8AndAHalf, maxMotorSpeed, 0.55f, 0.7f, null, 0.1f)
        ){

            @Override
            public boolean onExecuted() {

                return isTaskCompleted (0) || isTaskCompleted(1) || robot.straight();

            }

            @Override
            public void onReached() {
                super.onReached();

                if(robot.gyroSensor.getHeading() > 180){

                    robot.setDirection(CompbotHardware.MovementDirection.TURN_LEFT);

                }

                else if (robot.gyroSensor.getHeading() < 180){

                    robot.setDirection(CompbotHardware.MovementDirection.TURN_RIGHT);

                }

            }

        };

        final Goal<Integer> encoderGoal9 = new Goal<>(robot.inchesToEncoderTicks(62.5f));
        final ConcurrentTaskSet moveToSecondBeacon = new ConcurrentTaskSet(
                new MotorTask(robot.frontLeft, encoderGoal9, maxMotorSpeed, 0.45f, 0.7f, null, 0.1f),
                new MotorTask(robot.frontRight, encoderGoal9, maxMotorSpeed, 0.45f, 0.7f, null, 0.1f),
                new MotorTask(robot.backRight, encoderGoal9, maxMotorSpeed, 0.45f, 0.7f, null, 0.1f),
                new MotorTask(robot.backLeft, encoderGoal9, maxMotorSpeed, 0.45f, 0.7f, null, 0.1f)
        ) {

            long start;

            @Override
            public boolean onExecuted() {

                if(System.currentTimeMillis() - start >= 1000){

                    return isTaskCompleted (0) || isTaskCompleted(1) || robot.sensingWhite(robot.bottomRightColorSensor) || robot.sensingWhite(robot.bottomLeftColorSensor);


                }

                return false;

            }

            @Override
            public void onReached() {
                super.onReached();

                this.start = System.currentTimeMillis();

                robot.setDirection(CompbotHardware.MovementDirection.NORTH);

            }

            @Override
            public void onCompleted(){

                // happyTrail.addTask(calibrationStraif);
                happyTrail.addTask(calibrationBackUpSecondBeacon);
                happyTrail.addTask(isBlueTeam ? beaconHitBlue2 : beaconHitRed2);
                happyTrail.addTask(isBlueTeam ? backUpBlue : backUpRed);
                hitTheBeaconOnce = false;
                happyTrail.addTask(checkIfCorrectColor);

            }

        };

        checkIfCorrectColor = new Task() {
            boolean done = false;
            long startTime;

            // Called to update the task, return true if completed.
            public boolean execute (){

                if(startTime == 0){

                    return false;

                }

                if(System.currentTimeMillis() - startTime >= 5000){

                    done = true;

                }

                if(System.currentTimeMillis() - startTime >= 2000 && isColorGood()){

                    done = true;

                }

                if(numTimesHitBeacon > 2){

                    return true;

                }

                if(hitTheBeaconOnce){

                    return true;

                }

                return done;

            }

            // Returns if the task is complete. It no longer needs updates.
            public boolean isCompleted (){

                return done;

            }

            // Returns the TaskType of this task.
            // Mostly used for debugging as Task Type can be set to anything regardless of the actual task purpose.
            public TaskType getType (){

                return TaskType.WAIT;

            }

            // Event fired when the State Machine reaches this task.
            public void onReached (){

                startTime = System.currentTimeMillis();

            }

            // Event fired when the State Machine updates this task. Return true if task was completed while executing.
            public boolean onExecuted (){

                return done;

            }

            // Event fired when task completed
            public void onCompleted (){

                if (!isColorGood() && !hitTheBeaconOnce) {

                    hitTheBeaconOnce = true;

                    if (isBlueTeam) {
                        encoderGoal7 = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance));
                        encoderGoal7R = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance));
                        happyTrail.addTask(beaconHitBlue);
                        encoderGoal8 = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance - 0.5f));
                        encoderGoal8R = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance - 0.5f));
                        startTime = System.currentTimeMillis();
                        done = false;
                        happyTrail.addTask(backUpBlue);
                        happyTrail.addTask(this);

                    } else {

                        encoderGoal7 = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance));
                        encoderGoal7R = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance));
                        happyTrail.addTask(beaconHitRed);
                        encoderGoal8 = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance - 0.5f));
                        encoderGoal8R = new Goal<>(robot.inchesToEncoderTicks(buttonPressingDistance - 0.5f));
                        startTime = System.currentTimeMillis();
                        done = false;
                        happyTrail.addTask(backUpRed);
                        happyTrail.addTask(this);

                    }
                }

                else{

                    if(!onSecondBeacon){

                        happyTrail.clearTasks();
                        encoderGoal8 = new Goal<> (robot.inchesToEncoderTicks(buttonPressingDistance * 2f));
                        encoderGoal8R = new Goal<> (robot.inchesToEncoderTicks(buttonPressingDistance * 2f));
                        happyTrail.addTask(backUpBlue);
                        // happyTrail.addTask(align);
                        happyTrail.addTask(moveToSecondBeacon);
                        done = false;
                        startTime = System.currentTimeMillis();
                        hitTheBeaconOnce = false;

                    }

                    onSecondBeacon = true;

                }

            }

        };

        happyTrail.addTask(shoot);
        happyTrail.addTask(forward);
        happyTrail.addTask(isBlueTeam ? crabToLine1Blue : crabToLine1Red);
        happyTrail.addTask(isBlueTeam ? crabToLine2Blue : crabToLine2Red);
        happyTrail.addTask(calibrationBackUp);
        happyTrail.addTask(isBlueTeam ? beaconFirstApproachBlue : beaconFirstApproachRed);
        // happyTrail.addTask(isBlueTeam ? backUpBlue : backUpRed);
        // happyTrail.addTask(checkIfCorrectColor);

        addRoute(happyTrail);

        super.init();

    }

    private boolean isColorGood() {

        if(isBlueTeam){

            return robot.buttonPresserColorSensorRight.blue() > robot.buttonPresserColorSensorRight.red();

        }

        else{

            return robot.buttonPresserColorSensorLeft.red() > robot.buttonPresserColorSensorLeft.blue();

        }

    }

    @Override
    public void start () {

        robot.start();

        super.start();

    }

    @Override
    public void loop(){

        try{

            telemetry.addData("buttonPresserColorSensorRight", robot.buttonPresserColorSensorRight.red() + " " + robot.buttonPresserColorSensorRight.green() + " " + robot.buttonPresserColorSensorRight.blue());
            telemetry.addData("buttonPresserColorSensorLeft", robot.buttonPresserColorSensorLeft.red() + " " + robot.buttonPresserColorSensorLeft.green() + " " + robot.buttonPresserColorSensorLeft.blue());

            telemetry.addData("bottomRightColorSensor", robot.bottomRightColorSensor.red() + " " + robot.bottomRightColorSensor.green() + " " + robot.bottomRightColorSensor.blue());
            telemetry.addData("bottomLeftColorSensor", robot.bottomLeftColorSensor.red() + " " + robot.bottomLeftColorSensor.green() + " " + robot.bottomLeftColorSensor.blue());

            telemetry.addData("gyroSensor", robot.gyroSensor.getHeading());

            telemetry.addData("motor power (fr, fl, br, bl)", robot.frontRight.getPower() + " " + robot.frontLeft.getPower() + " " + robot.backRight.getPower() + " " + robot.backLeft.getPower());

            super.loop();

        }catch(NullPointerException e){

            LoggedOp.debugOut = "NULL :(";

        }

    }

}