/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import static com.pedropathing.api.Paths.line;
import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;
import java.util.Collections;
import java.util.Map;
import org.junit.jupiter.api.Test;

class FollowerImmediateStopTest {
    @Test
    void stopImmediatelyClearsTheLastManualMotorCommand() {
        DriveStub drive = new DriveStub();
        Follower follower = new Follower(new LocalizerStub(), drive, new AlgorithmStub());
        follower.manual(0.7, -0.3, 0.2);
        follower.update(0.02);

        follower.stop();

        assertTrue(follower.idle());
        assertEquals(0.0, drive.powers.forward());
        assertEquals(0.0, drive.powers.strafe());
        assertEquals(0.0, drive.powers.turn());
        assertEquals(1, drive.stopCalls);
    }

    @Test
    void stoppingReleasesActivePathModifiersWithoutAnotherUpdate() {
        DriveStub drive = new DriveStub();
        Follower follower = new Follower(new LocalizerStub(), drive, new AlgorithmStub());
        com.pedropathing.config.ConfigVar<Double> speed = com.pedropathing.config.ConfigVar.of(1.0);
        follower.follow(line(Pose.zero(), new Pose(10, 0)).constant(0).with(speed.at(0.5)));
        assertEquals(0.5, speed.get());

        follower.stop();

        assertEquals(1.0, speed.get());
        assertNull(follower.currentPath());
        assertTrue(follower.idle());
        assertEquals(1, drive.stopCalls);
    }

    private static final class LocalizerStub implements Localizer {
        private MotionState state = MotionState.zero();

        @Override
        public void setPose(Pose pose) {
            state = state.withPose(pose);
        }

        @Override
        public MotionState state() {
            return state;
        }

        @Override
        public void update() {}

        @Override
        public void reset() {
            state = MotionState.zero();
        }
    }

    private static final class DriveStub implements Drivetrain {
        private DrivePowers powers = DrivePowers.zero();
        private int stopCalls;

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            this.powers = powers;
        }

        @Override
        public void stop() {
            powers = DrivePowers.zero();
            stopCalls++;
        }

        @Override
        public void stop(boolean brake) {
            stop();
        }

        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            return 1;
        }

        @Override
        public double interpolateVelocity(double x, double y, double theta) {
            return x;
        }

        @Override
        public Map<String, Object> debug() {
            return Collections.emptyMap();
        }
    }

    private static final class AlgorithmStub implements Algorithm {
        private RuntimeException resetFailure;

        @Override
        public DrivePowers calculatePath(Drivetrain d, PathTracker p, MotionState s, double dt) {
            return DrivePowers.zero();
        }

        @Override
        public DrivePowers calculateHold(Drivetrain d, Pose p, MotionState s, boolean scale, double dt) {
            return DrivePowers.zero();
        }

        @Override
        public double completion() {
            return 0;
        }

        @Override
        public Pose closestPose() {
            return Pose.zero();
        }

        @Override
        public Vector2D closestTangent() {
            return Vector2D.zero();
        }

        @Override
        public Vector2D closestNormal() {
            return Vector2D.zero();
        }

        @Override
        public double curvature() {
            return 0;
        }

        @Override
        public double remainingDistance() {
            return 0;
        }

        @Override
        public boolean atParametricEnd() {
            return false;
        }

        @Override
        public void reset() {
            if (resetFailure != null) throw resetFailure;
        }

        @Override
        public boolean isBusy() {
            return true;
        }

        @Override
        public Map<String, Object> debug() {
            return Collections.emptyMap();
        }
    }
}
