package org.firstinspires.ftc.teamcode.commands
class ForeverCommand(f: () -> Command, name: String): RepeatUntil(f, {false}, name)
fun ForeverCommand(f: () -> Command) = ForeverCommand(f, "Forever")
fun Forever(f: () -> Unit) = ForeverCommand{ Instant(f) }