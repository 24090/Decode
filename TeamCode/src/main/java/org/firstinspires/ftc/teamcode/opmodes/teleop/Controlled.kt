package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import org.firstinspires.ftc.teamcode.opmodes.poses.scorePosition
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.SquID
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
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
        val driveFunction = {
            drive.strafe = -gamepad1.left_stick_x.toDouble();
            drive.drive = -gamepad1.left_stick_y.toDouble();
            drive.turn = -gamepad1.right_stick_x.toDouble();
            drive.setMotorPowers()
            drive.localizer.update()
        }
        waitForStart()
        drive.localizer.pose = Pose(12.0, 12.0, PI)
        while (opModeIsActive()){
            bulkReads.update()
            driveFunction()
            shooter.setTargetVelocityFromDistance((scorePosition - Vector.fromPose(drive.localizer.pose)).length)
            shooter.update()
            intake.update()
            telemetry.addData("pose", drive.localizer.pose)
            telemetry.addData("distance", (scorePosition - Vector.fromPose(drive.localizer.pose)).length)
            telemetry.addData("Shooter velocity", shooter.currentVelocity)
            telemetry.addData("Target velocity", shooter.velocity)
            telemetry.update()
            if (gamepad1.aWasReleased()) {
                runBlocking(intake.spinUp())
            }
            if (gamepad1.bWasReleased()) {
                runBlocking(intake.spinDown())
            }
            if (gamepad1.xWasReleased()) {
                runBlocking(Race(
                    Forever {
                        bulkReads.update()
                        drive.localizer.update()

                        intake.update()

                        val relativePose = (Vector.fromPose(drive.localizer.pose) - scorePosition)

                        shooter.setTargetVelocityFromDistance(relativePose.length)
                        shooter.update()

                        drive.targetPose = Pose(
                            drive.localizer.x, drive.localizer.y,
                            relativePose.angle
                        )
                        drive.strafe = -gamepad1.left_stick_x.toDouble();
                        drive.drive = -gamepad1.left_stick_y.toDouble();
                        drive.turn = SquID(AngleUnit.normalizeRadians(drive.error.heading), Drive.DriveConstants.kSqH)
                        drive.setMotorPowers()

                        telemetry.addData("Shooter velocity", shooter.currentVelocity)
                        telemetry.update()
                    },
                    Sequence(
                        WaitUntil { drive.error.heading < 0.04 },
                        shooter.waitForVelocity(),
                        intake.releaseBall()
                    )
                ))
            }
        }
    }

}