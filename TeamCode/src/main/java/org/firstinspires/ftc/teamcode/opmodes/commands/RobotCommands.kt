package org.firstinspires.ftc.teamcode.opmodes.commands

import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Pattern

fun shootCycle(intake: Intake, shooter: Shooter) = Sequence(
        Parallel(
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
        ),
        intake.releaseDual(),
        Sleep(0.3),
        Parallel(
            intake.moveInThird(),
            shooter.waitForVelocity(),
        ),
        intake.releaseDual(),
        name = "ShootCycle"
    )


fun releasePattern(intake: Intake, shooter: Shooter, huskyLens: HuskyLens, indexTracker: IndexTracker): Command = Future {
    huskyLens.read()
    val held = huskyLens.getHeldPattern()
    val pattern = indexTracker.getRecommendations()
    return@Future Sequence (
        Parallel(
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
        ),
        when (Pair(held, pattern)){
            Pair(Pattern.GPP, Pattern.GPP) -> Sequence(
                // left right+3
                intake.releaseLeft(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.GPP, Pattern.PGP) -> Sequence(
                // right left 3
                intake.releaseRight(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseLeft(),
                Parallel(Sleep(0.5), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.GPP, Pattern.PPG) -> Sequence(
                // right 3 left
                intake.releaseRight(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseRight(),
                Parallel(Sleep(0.5), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PGP, Pattern.GPP) -> Sequence(
                // right left+3
                intake.releaseRight(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PGP, Pattern.PGP) -> Sequence(
                // left right 3
                intake.releaseLeft(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseRight(),
                Parallel(Sleep(0.5), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PGP, Pattern.PPG), Pair(Pattern.PPG, Pattern.PGP) -> Sequence(
                // left 3 right
                intake.releaseLeft(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseLeft(),
                Parallel(Sleep(0.5), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PPG, Pattern.GPP), Pair(Pattern.PPG, Pattern.PPG) -> Sequence(
                // BAD/GOOD
                // left+right 3
                intake.releaseDual(),
                Sleep(pusherWait),
                Parallel( intake.moveInThird(), shooter.waitForVelocity()),
                intake.releaseDual(),
            )

            else -> {throw UnsupportedOperationException("Unreachable")}
        }
    )
}