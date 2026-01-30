package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp
class ShootCycleTesting: LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        val reads = Reads(hardwareMap)
        shooter.motorLeft.power = 0.28
        shooter.motorRight.power = 0.28
        intake.behaviour = Intake.IntakeBehaviour.Grab
        intake.update()
        waitForStart()
        runBlocking(Race(
            Forever {
                reads.update()
                intake.update()
            },
            Sequence(
                Parallel(
                    intake.releaseDual(),
                    intake.setAdjustThird()
                ),
                Sleep(pusherWait),
                Parallel(
                    intake.spinUp(),
                    Sleep(0.3),
                ),
                intake.releaseDual(),
            )
        ))
    }

}