package org.firstinspires.ftc.teamcode.commands
class ForeverCommand(f: () -> Command, name: String): RepeatUntil(f, {false}, name)

fun ForeverCommand(f: () -> Command) = ForeverCommand(f, "Forever")
class Forever(val f: () -> Unit, name: String = "Forever"): OverrideButtonCommand(name) {
    override fun run(): CommandResult {
        f.invoke()
        return CommandResult.Continue
    }
}

fun Forever(f: () -> Unit) = Forever(f, "Forever")
