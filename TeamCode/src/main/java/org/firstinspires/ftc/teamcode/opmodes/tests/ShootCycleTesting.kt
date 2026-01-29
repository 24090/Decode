package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp
class ShootCycleTesting: LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val shooter = Shooter(hardwareMap)
        shooter.motorLeft.power = 0.25
        shooter.motorRight.power = 0.25
        intake.spinUp().update()
        intake.update()
        waitForStart()
        runBlocking(Race(
            Forever {
                intake.update()
                shooter.update()
            },
            Sequence (
                intake.releaseDual(),
                Sleep(pusherWait),
                Parallel(
                    Sleep(0.15)
                ),
                intake.releaseDual(),
            ),
        ))
    }

}