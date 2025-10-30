package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherLeftForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightBack
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherRightForward
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.BulkReads
import kotlin.math.PI

@TeleOp(name="Controlled")
class Controlled: LinearOpMode() {
    override fun runOpMode() {
        val bulkReads = BulkReads(hardwareMap)
        val drive = Drive(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val intake = Intake(hardwareMap)
        val updateHeadingOverride = {
            drive.turn = -gamepad1.right_stick_x.toDouble();
        }
        val updateTranslationalOverride = {
            drive.strafe = -gamepad1.left_stick_x.toDouble();
            drive.drive = -gamepad1.left_stick_y.toDouble();
        }
        drive.currentUpdateHeading = updateHeadingOverride
        drive.currentUpdateTranslational = updateTranslationalOverride
        waitForStart()
        drive.localizer.pose = Pose(12.0, 12.0, 0.0)
        while (opModeIsActive()){
            bulkReads.update()
            drive.update()
            shooter.setTargetVelocityFromDistance((scorePosition - Vector.fromPose(drive.localizer.pose)).length)
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("distance", (scorePosition - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Target velocity", shooter.targetVelocity)
            telemetry.update()
            if (gamepad1.aWasPressed()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasPressed()) {
                runBlocking(intake.spinDown())
            }
            if (gamepad1.leftBumperWasPressed()) {
                runBlocking(
                    ForeverCommand{
                        Sequence (
                            Instant{
                                intake.pusherLeft.position = pusherLeftForward
                                intake.pusherRight.position = pusherRightBack
                            },
                            Sleep(pusherWait),
                            Instant{
                                intake.pusherRight.position = pusherRightForward
                                intake.pusherLeft.position = pusherLeftBack
                            },
                            Sleep(pusherWait),
                        )
                    }
                )
            }
            if (gamepad1.rightBumperWasPressed()) {
                runBlocking(intake.releaseRight())
            }
            if (gamepad1.xWasPressed()) {
                drive.currentUpdateHeading = drive::updateHeading
                runBlocking(Race(
                    Forever {
                        bulkReads.update()
                        intake.update()
                        drive.localizer.update()
                        val relativePose = (scorePosition - Vector.fromPose(drive.localizer.pose))
                        drive.targetPose = Pose(
                            drive.localizer.x, drive.localizer.y,
                            relativePose.angle
                        )
                        drive.update(updateLocalizer = false)
                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        shooter.update()
                        telemetry.addData("Shooter velocity", (shooter.motorLeft.velocity + shooter.motorRight.velocity)/2)
                        telemetry.addData("Target velocity", shooter.targetVelocity)
                        telemetry.update()
                    },
                    Sequence(
                        WaitUntil { drive.error.heading < 0.04 },
                        shooter.waitForVelocity(),
                        intake.releaseDual()
                    )
                ))
                drive.currentUpdateHeading = updateHeadingOverride
            }
        }
    }

}