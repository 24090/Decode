package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.Forever
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.calculatePredictiveMoveShoot

@TeleOp
class MoveShootTest: LinearOpMode(){
    override fun runOpMode() {
        val reads = Reads(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val drive = Drive(hardwareMap)
        reads.update()
        var time = System.currentTimeMillis()
        val recordTime = { name:String ->
            val newTime = System.currentTimeMillis()
            telemetry.addData("$name (ms)", newTime - time)
            time = newTime
        }
        waitForStart()
        drive.localizer.pose = startPose
        var outputs: Pair<Double, Double>? = null
        drive.currentUpdateTranslational = {
            val v = Vector.fromCartesian(-gamepad1.left_stick_x.toDouble(), gamepad1.left_stick_y.toDouble())
            drive.strafe = v.rotated(-drive.localizer.heading).y
            drive.drive =  v.rotated(-drive.localizer.heading).x
        }
        drive.currentUpdateHeading = {
            drive.targetPose.heading = outputs?.second ?: drive.targetPose.heading
            drive.updateHeading()
        }
        var nextShoot = System.currentTimeMillis() + pusherWait * 1000
        runBlocking(
            Forever {
                if (gamepad1.xWasPressed() || !gamepad1.x){
                    nextShoot = System.currentTimeMillis() + pusherWait * 1000
                }
                reads.update()
                outputs = calculatePredictiveMoveShoot((System.currentTimeMillis() - nextShoot)/1000, drive.localizer.pose, drive.localizer.poseVel)
                recordTime("reads")
                drive.update()
                telemetry.addData("target exit velocity", outputs?.first)
                telemetry.addData("target heading", outputs?.second)
                telemetry.addData("pose", drive.localizer.pose)
                telemetry.addData("pose", drive.localizer.poseVel)
                telemetry.addData("Target velocity: ", "L: ${shooter.targetVelocityLeft}, R: ${shooter.targetVelocityRight}")
                telemetry.addData("motorLeftVelocity", shooter.motorLeft.velocity)
                telemetry.addData("motorRightVelocity", shooter.motorRight.velocity)
                intake.pusherLeft.position = if (gamepad1.x) pusherLeftForward else pusherLeftBack
                intake.pusherRight.position = if (gamepad1.x) pusherRightForward else pusherRightBack
                shooter.targetVelocityRight = shooter.exitVelocityToRightVelocityLUT.get(outputs?.first ?: 1350.0)
                shooter.targetVelocityLeft = shooter.exitVelocityToLeftVelocityLUT.get(outputs?.first ?: 1300.0)
                shooter.update(); recordTime("shooter")
                intake.update(); recordTime("intake")
                telemetry.update()
            }
        )
    }
}