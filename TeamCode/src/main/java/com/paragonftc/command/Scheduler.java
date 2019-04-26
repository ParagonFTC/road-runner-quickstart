package com.paragonftc.command;

/**
 * The Scheduler registers subsystems and runs commands for those subsystems
 */
public interface Scheduler {
    void register(Subsystem subsystem);

    void run(Command command);

    void update();

    void kill();
}
