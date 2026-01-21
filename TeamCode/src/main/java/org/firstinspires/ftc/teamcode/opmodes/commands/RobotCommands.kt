package org.firstinspires.ftc.teamcode.opmodes.commands

import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pointToPoint
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Pattern
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

fun moveShootAll(intake: Intake, shooter: Shooter, getHeading: () -> Double, getMoveShootOutputs: () -> Pair<Double, Double>?) = Sequence(
    intake.fullAdjustThird(),
    WaitUntil {
            abs(getHeading() - (getMoveShootOutputs()?.second ?: 10.0)) < 0.06
            && abs(shooter.targetVelocityRight - shooter.motorRight.velocity) < 50
            && abs(shooter.targetVelocityLeft - shooter.motorLeft.velocity) < 50
    },
    intake.releaseDual(),
    Sleep(pusherWait),
    intake.spinUp(),
    Sleep(1.0),
    WaitUntil {
        abs(getHeading() - (getMoveShootOutputs()?.second ?: 10.0)) < 0.06
        && abs(shooter.targetVelocityRight - shooter.motorRight.velocity) < 50
        && abs(shooter.targetVelocityLeft - shooter.motorLeft.velocity) < 50
    },
    intake.releaseDual(),
)
fun shootAll(intake: Intake, shooter: Shooter) = Sequence(
        Parallel(
            intake.fullAdjustThird(),
            shooter.waitForVelocity(),
        ),
        intake.releaseDual(),
        Sleep(pusherWait),
        Parallel(
            intake.spinUp(),
            shooter.waitForVelocity(),
            Sleep(0.3),
        ),
        intake.releaseDual(),
        name = "ShootCycle"
    )
fun shootPattern(intake: Intake, shooter: Shooter, huskyLens: HuskyLens, indexTracker: IndexTracker): Command = Future {
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
    Instant {
        drive.follow = {
            val distanceX = (36.0 + 24.0 * n) - drive.localizer.x
            val targetPose = Pose(36.0 + 24.0 * n, (if (n == 2) 75.0 else 80.0) - min(distanceX * 2, 60.0), PI/2).mirroredIf(isRed)
            pointToPoint(drive.localizer.pose, drive.localizer.poseVel, targetPose)
        }
    },
    Race(
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
    if (n == 1) {
        drive.goToSquare(Pose(36.0 + 24.0 * n, 24.0, PI/2).mirroredIf(isRed), yTolerance = 3.0, xTolerance = 3.0)
    } else {
        Instant {}
    },
    name = "GrabBallCycle $n"
)

fun fromRampCycle(isRed: Boolean, intake: Intake, drive: Drive) = Sequence(
    Race(
        Sequence(
            Sleep(0.3),
            WaitUntil{drive.localizer.poseVel.vector().length < 0.5}
        ),
        drive.goToCircle(
            Pose(
                62.06,
                60.8,
                0.875
            ).mirroredIf(isRed),
        ),
    ),
    Race(
        Sequence(
            intake.waitForStall(),
            Sleep(0.3),
            intake.waitForStall()
        ),
        Sleep(4.0),
        drive.goToCircle(Pose(
            52.8,
            59.55,
            0.285
        ).mirroredIf(isRed)
        )
    )
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
                72.0 - robotLength / 2.0,
                PI / 2.0
            ).mirroredIf(isRed),
        )
    )
)