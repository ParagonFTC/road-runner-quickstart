package com.paragonftc.command;

public class SequentialCommandGroup implements Command {
    private Command[] commands;
    private int currentIndex = 0;


    public SequentialCommandGroup(Command... commands) {
        this.commands = commands;
    }
    @Override
    public boolean isCompleted() {
        return currentIndex >= commands.length;
    }

    @Override
    public void start() {
        commands[0].start();
    }

    @Override
    public void update() {
        if (currentIndex < commands.length) {
            Command currentCommand = commands[currentIndex];

            if (currentCommand.isCompleted()) {
                currentCommand.end();
                currentIndex ++;

                if (currentIndex < commands.length) {
                    commands[currentIndex].start();
                }
            } else {
                currentCommand.update();
            }
        }
    }

    @Override
    public void end() {
        commands[currentIndex].end();
    }
}
