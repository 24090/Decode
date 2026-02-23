package org.firstinspires.ftc.teamcode.opmodes.commands

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Future
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Parallel
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.commands.runBlocking
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.farPose
import org.firstinspires.ftc.teamcode.opmodes.poses.getScorePose
import org.firstinspires.ftc.teamcode.opmodes.poses.parkPose
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.pointToPoint
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.huskylens.Lights
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake.Params.pusherWait
import org.firstinspires.ftc.teamcode.subsystems.reads.Reads
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.vision.Camera
import org.firstinspires.ftc.teamcode.util.IndexTracker
import org.firstinspires.ftc.teamcode.util.Observation
import org.firstinspires.ftc.teamcode.util.Pattern
import org.firstinspires.ftc.teamcode.util.Reference
import org.firstinspires.ftc.teamcode.util.clamp
import org.firstinspires.ftc.teamcode.util.storedRed
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min

open class Robot(hwMap: HardwareMap, val telemetry: Telemetry) {
    var red
        get() = storedRed.get()
        set(v) = storedRed.set(v)
    val reads = Reads(hwMap)
    val drive = Drive(hwMap)
    val shooter = Shooter(hwMap)
    val intake = Intake(hwMap)
    val huskyLens = HuskyLens(hwMap)
    val camera = Camera(hwMap)
    val lights = Lights(hwMap)
    val indexTracker = IndexTracker()

    var time = System.currentTimeMillis()
    fun recordTime(name: String) {
        val newTime = System.currentTimeMillis()
        telemetry.addData("$name (ms)", newTime - time)
        time = newTime
    }

    fun moveShootAll(getHeading: () -> Double, getMoveShootOutputs: () -> Pair<Double, Double>?) =
        org.firstinspires.ftc.teamcode.commands.Sequence(
            intake.fullAdjustThird(),
            WaitUntil {
                abs(getHeading() - (getMoveShootOutputs()?.second ?: 10.0)) < 0.06
                && shooter.velocitiesInThreshold(50.0)
            },
            intake.releaseDual(),
            Sleep(pusherWait),
            intake.spinUp(),
            Sleep(1.0),
            WaitUntil {
                abs(getHeading() - (getMoveShootOutputs()?.second ?: 10.0)) < 0.06
                && shooter.velocitiesInThreshold(50.0)
            },
            intake.releaseDual(),
        )
    fun shootAll() = org.firstinspires.ftc.teamcode.commands.Sequence(
        shooter.waitForVelocity(),
        Parallel(
            intake.releaseDual(),
            intake.setAdjustThird()
        ),
        Parallel(
            Instant { intake.behaviour = Intake.IntakeBehaviour.Greedy },
            Sleep(pusherWait),
        ),
        Parallel(
            shooter.waitForVelocity(),
            Instant { intake.behaviour = Intake.IntakeBehaviour.Greedy },
            Sleep(0.3),
        ),
        intake.releaseDual(),
        Instant { intake.behaviour = Intake.IntakeBehaviour.Grab },
        name = "ShootCycle"
    )
    fun shootPattern(): Command = Future {
        huskyLens.update()
        val reload = {
            Parallel(
                shooter.waitForVelocity(),
                intake.spinUp(),
                Sleep(0.3),
            )
        }
        val held = huskyLens.getHeldPattern()
        val pattern = indexTracker.getRecommendations()
        lights.turnOn()
        //val startCount = shooter.shootCounterLeft.count + shooter.shootCounterRight.count
        val indexWait = 0.75
        val command = Sequence(
            Parallel(
                intake.setAdjustThird(),
                shooter.waitForVelocity(),
            ),
            when (Pair(held, pattern)) {
                Pair(Pattern.GPP, Pattern.GPP) -> Sequence(
                    // left right+3
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.GPP, Pattern.PGP) -> Sequence(
                    // right left 3
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.GPP, Pattern.PPG) -> Sequence(
                    // right 3 left
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PGP, Pattern.GPP) -> Sequence(
                    // right left+3
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PGP, Pattern.PGP) -> Sequence(
                    // left right 3
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseRight(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PGP, Pattern.PPG), Pair(Pattern.PPG, Pattern.PGP) -> Sequence(
                    // left 3 right
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseLeft(),
                    Parallel(reload(), Sleep(indexWait)),
                    intake.releaseDual()
                )

                Pair(Pattern.PPG, Pattern.GPP), Pair(Pattern.PPG, Pattern.PPG) -> Sequence(
                    // BAD/GOOD
                    // left+right 3
                    intake.releaseDual(),
                    reload(),
                    intake.releaseDual(),
                )

                else -> {
                    throw UnsupportedOperationException("Unreachable")
                }
            },
            Instant {
                lights.turnOff()
            }
//            Instant {
//                indexTracker.processObservation(Observation.Shot(shooter.shootCounterLeft.count + shooter.shootCounterRight.count - startCount))
//            }
        )
        return@Future command
    }
    fun grabAndOpenCycleClose() = Sequence(
        intake.spinUp(),
        Race(
            drive.followPath(PurePursuitPath(
                listOf(Pose(58.0, 25.0, PI/2.0), Pose(58.0, 65.0, PI/2.0)),
                listOf(HeadingBehaviour.Tangent(0.0)),
                listOf(30.0)
            )),
            WaitUntil { intake.isStalling() && drive.localizer.pose.mirroredIf(red).y > 48 },
            WaitUntil{ drive.localizer.poseVel.vector().length < 0.2 && drive.localizer.pose.mirroredIf(red).y > 48}
        ),
        drive.goToSquare(Pose(62.0, 48.0, PI/2.0).mirroredIf(red), 1.5, 1.5, 0.1),
        Race(
            drive.goToSquare(Pose(65.0, 54.0, PI/2.0).mirroredIf(red)),
            Sleep(1.75)
        ),
        drive.followPath(PurePursuitPath(
            listOf(closePose, Pose(58.0, 25.0, PI/2.0)),
            listOf(HeadingBehaviour.Tangent(0.0)),
            listOf(30.0)
        ), 5.0, 0.1),
    )
    fun grabAndOpenCycleFar() = Sequence(
        intake.spinUp(),
        Instant {
            drive.follow = {
                val distanceX = abs((42.0 + 24.0) - drive.localizer.x)
                val targetPose = Pose(
                    66.0,
                    80.0 - min(distanceX * 3, 60.0),
                    PI/2
                ).mirroredIf(red)
                pointToPoint(drive.localizer.pose, drive.localizer.poseVel, targetPose)
            }
        },
        Race(
            Sequence(
                WaitUntil { intake.isStalling() && drive.localizer.pose.mirroredIf(red).y > 48 }
            ),
            Sequence(
                WaitUntil{ drive.localizer.poseVel.vector().length < 0.2 && drive.localizer.pose.mirroredIf(red).y > 48},
            ),
        ),
        Instant {
            indexTracker.processObservation(Observation.GateOpened)
            drive.follow = {
                val distanceY = abs(closePose.y - drive.localizer.y)
                val targetPose = Pose(
                    closePose.x - clamp(distanceY, 0.0, 16.0),
                    closePose.y,
                    closePose.heading * clamp(distanceY/20.0, 0.0, 1.0) + PI/2 * (1 - clamp(distanceY/20.0, 0.0, 1.0))
                ).mirroredIf(red)
                pointToPoint(drive.localizer.pose, drive.localizer.poseVel, targetPose)
            }
        },
        drive.goToSquare(Pose(38.0 + 24.0, 24.0, PI/2).mirroredIf(red), yTolerance = 3.0, xTolerance = 3.0)
    )
    fun grabBallCycle (n: Int, endPark: Boolean = false): Sequence{
        val endPoint = Pose(
            (if (n == 1) 34.0 else 36.0) + 24.0 * n,
            (65.0),
            PI/2
        )
        val shootPoint = if (endPark) parkPose else closePose
        return Sequence(
            intake.spinUp(),

            Race(
                drive.followPath(PurePursuitPath(
                    listOf(
                        Pose(
                            endPoint.x,
                            30.0,
                            PI/2
                        ).mirroredIf(red),
                        endPoint.mirroredIf(red)
                    ),
                    listOf(HeadingBehaviour.Tangent(0.0)),
                    listOf(30.0)
                )),
                WaitUntil { intake.isStalling() && drive.localizer.pose.mirroredIf(red).y > 48 },
                WaitUntil{ drive.localizer.poseVel.vector().length < 1.0 && drive.localizer.pose.mirroredIf(red).y > 48},
            ),
            Race(
                drive.followPath(PurePursuitPath(
                    listOf(
                        Pose(endPoint.x, if (n == 1) 20.0 else 40.0, PI/2).mirroredIf(red),
                        shootPoint.mirroredIf(red)
                    ),
                    listOf(
                        HeadingBehaviour.Interpolate,
                    ),
                    listOf(
                        30.0
                    ),
                )),
                WaitUntil{drive.localizer.pose.inCircle(shootPoint.mirroredIf(red), 5.0, 1.0)},
            ),
            name = "GrabBallCycle $n"
        )
    }

    fun fromRampCycle() = Sequence(
        Race(
            Sequence(
                Parallel(
                    Sleep(0.3),
                ),
                WaitUntil{drive.localizer.poseVel.vector().length < 0.5},
            ),
            drive.goToCircle(
                Pose(
                    59.4,
                    56.12 + 2.5,
                    1.06
                ).mirroredIf(red),
            ),
        ),
        Race(
            Sequence(
                intake.waitForStall(),
                intake.waitForStall(),
                intake.waitForStall(),
            ),
            Sequence(
                Instant {intake.behaviour = Intake.IntakeBehaviour.SlowIntake},
                Sleep(1.8)
            )

        ),
        Instant {
            indexTracker.processObservation(Observation.GateOpened)
            intake.behaviour = Intake.IntakeBehaviour.Grab
        },
        Race(
            drive.followPath(PurePursuitPath(
                listOf(
                    Pose(
                        59.4,
                        56.12 + 2.5,
                        1.06
                    ).mirroredIf(red),
                    Pose(
                        59.4,
                        35.0,
                        PI/2
                    ).mirroredIf(red),
                    closePose
                ),
                listOf(
                    HeadingBehaviour.Tangent(PI),
                    HeadingBehaviour.Interpolate
                ),
                listOf(
                    20.0,
                    20.0
                )
            )),
            WaitUntil {drive.localizer.pose.inCircle(closePose, 5.0, 0.1)}
        )
    )

    fun loadZoneCycle() = Race(
        Sequence(
            intake.spinUp(),
            Sleep(2.0),
            intake.waitForStall()
        ),
        Sequence(
            Sleep(2.0),
            WaitUntil { drive.localizer.poseVel.vector().length < 0.1}
        ),
        Sequence(
            drive.goToCircle(
                Pose(
                    robotWidth / 2.0 + 0.5,
                    72.0 - robotLength / 2.0 + 1.0,
                    PI / 2.0
                ).mirroredIf(red),
            )
        )
    )

    fun closeShootCyclePattern() = Sequence(
        Instant{lights.turnOn()},
        drive.goToCircle(closePose.mirroredIf(red), 2.0),
        shootPattern(),
        Instant{lights.turnOff()},
        name = "CloseShootCycle"
    )

    fun closeShootCycle() = Sequence(
        drive.goToCircle(closePose.mirroredIf(red), 2.0),
        shootAll(),
        name = "CloseShootCycle"
    )

    fun farShootCycle() = Sequence(
        drive.goToCircle(farPose.mirroredIf(red), 2.0),
        shootAll(),
        name = "FarShootCycle"
    )

    fun leaveShootCyclePattern() = Sequence(
        Instant{lights.turnOn()},
        drive.goToCircle(getScorePose(Vector.fromCartesian(106.0, 12.0)).mirroredIf(storedRed.get()), 2.0),
        shootPattern(),
        Instant{lights.turnOff()},
        name = "CloseShootCycle"
    )

    fun leaveShootCycle() = Sequence(
        //Instant{lights.turnOn()},
        drive.goToCircle(getScorePose(Vector.fromCartesian(106.0, 12.0)).mirroredIf(storedRed.get()), 2.0),
        shootAll(),
        //Instant{lights.turnOff()},
        name = "CloseShootCycle"
    )
}

open class Auto(val red: Boolean, val startPose: Pose, val command: Robot.() -> Command) : LinearOpMode(){
    override fun runOpMode() {
        storedRed = Reference(red)
        val robot = Robot(hardwareMap, telemetry)
        val command = robot.command()
        while (opModeInInit()){
            robot.indexTracker.pattern = robot.camera.getPattern() ?: robot.indexTracker.pattern
            telemetry.addData("pattern", robot.indexTracker.pattern)
            telemetry.update()
        }

        waitForStart()

        if (!opModeIsActive()){
            return
        }

        robot.drive.localizer.pose = startPose.mirroredIf(storedRed.get())
        robot.drive.startP2PWithTargetPose(closePose.mirroredIf(storedRed.get()))
        robot.camera.sendHeading(startPose.heading)
        runBlocking(command)
    }

}

open class Teleop(val function: Robot.(opmode: LinearOpMode) -> Unit): LinearOpMode(){
    override fun runOpMode() {
        val robot = Robot(hardwareMap, telemetry)
        robot.function(this)
    }
}
