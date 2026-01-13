package org.firstinspires.ftc.teamcode.opmodes.commands

import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Pattern
import kotlin.math.PI
import kotlin.math.abs
fun shootCycle(intake: Intake, shooter: Shooter) = Sequence(
        Parallel(
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
        ),
        intake.releaseDual(),
        Sleep(0.3),
        Parallel(
            intake.spinUp(),
            shooter.waitForVelocity(),
            Sleep(0.3),
        ),
        intake.releaseDual(),
        name = "ShootCycle"
    )
fun releasePattern(intake: Intake, shooter: Shooter, huskyLens: HuskyLens, indexTracker: IndexTracker): Command = Future {
    huskyLens.read()
    val held = huskyLens.getHeldPattern()
    val pattern = indexTracker.getRecommendations()
    val indexWait = 1.0
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
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.GPP, Pattern.PGP) -> Sequence(
                // right left 3
                intake.releaseRight(),
                Sleep(pusherWait),
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseLeft(),
                Parallel(Sleep(indexWait), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.GPP, Pattern.PPG) -> Sequence(
                // right 3 left
                intake.releaseRight(),
                Sleep(pusherWait),
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseRight(),
                Parallel(Sleep(indexWait), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PGP, Pattern.GPP) -> Sequence(
                // right left+3
                intake.releaseRight(),
                Sleep(pusherWait),
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PGP, Pattern.PGP) -> Sequence(
                // left right 3
                intake.releaseLeft(),
                Sleep(pusherWait),
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseRight(),
                Parallel(Sleep(indexWait), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PGP, Pattern.PPG), Pair(Pattern.PPG, Pattern.PGP) -> Sequence(
                // left 3 right
                intake.releaseLeft(),
                Sleep(pusherWait),
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseLeft(),
                Parallel(Sleep(indexWait), shooter.waitForVelocity()),
                intake.releaseDual()
            )
            Pair(Pattern.PPG, Pattern.GPP), Pair(Pattern.PPG, Pattern.PPG) -> Sequence(
                // BAD/GOOD
                // left+right 3
                intake.releaseDual(),
                Sleep(pusherWait),
                Parallel( intake.fullAdjustThird(), shooter.waitForVelocity()),
                intake.releaseDual(),
            )

            else -> {throw UnsupportedOperationException("Unreachable")}
        }
    )
}
fun grabBallCycle (n: Int, isRed: Boolean, intake: Intake, drive: Drive) = Sequence(
    intake.spinUp(),
    drive.goToSquare(Pose(36.0 + 24.0 * n, 24.0, PI/2).mirroredIf(isRed)),
    Instant {Drive.DriveConstants.xyP /= 6},
    Race(
        drive.goToCircle(Pose(
            36.0 + 24.0 * n,
            if (n == 2) 75.0 else 80.0,
            PI/2
        ).mirroredIf(isRed)),
        Sequence(
            intake.waitForStall(),
            Sleep(0.2),
            intake.waitForStall()
        ),
        Sequence(
            WaitUntil{ abs(drive.localizer.yVel) < 0.3 },
            Sleep(0.2),
            WaitUntil{ abs(drive.localizer.yVel) < 0.3 }
        ),
    ),
    Instant {Drive.DriveConstants.xyP *= 6},
    if (n == 1) {
        drive.goToSquare(Pose(36.0 + 24.0 * n, 24.0, PI/2).mirroredIf(isRed))
    } else {
        Instant {}
    },
    name = "GrabBallCycle $n"
)

fun loadZoneCycle(isRed: Boolean, intake: Intake, drive: Drive) = Race(
    Sequence(
        intake.spinUp(),
        Sleep(0.3),
        intake.waitForStall()
    ),
    Sequence(
        Sleep(0.3),
        WaitUntil { drive.localizer.poseVel.vector().length < 0.1}
    ),
    Sequence(
        drive.goToCircle(
            Pose(
                robotWidth / 2.0,
                60.0,
                PI / 2.0
            ).mirroredIf(isRed), 4.0
        )
    )
)