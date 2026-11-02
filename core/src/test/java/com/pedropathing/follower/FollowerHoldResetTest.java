/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.follower;

import static com.pedropathing.api.Paths.line;
import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import java.util.Collections;
import java.util.Map;
import org.junit.jupiter.api.Test;

class FollowerHoldResetTest {
    @Test
    void freshHoldIsBusyWhileDrivingAndClearsWhenItSettles() {
        LocalizerStub localizer = new LocalizerStub();
        DriveStub drive = new DriveStub();
        Follower follower = new Follower(localizer, drive, new Foresight(config()));
        Pose target = new Pose(10, 0);

        follower.hold(target);

        assertTrue(follower.isBusy());
        follower.update(0.02);
        assertTrue(follower.isBusy());
        assertTrue(drive.powers.forward() > 0);
        localizer.setPose(target);
        follower.update(0.02);
        assertFalse(follower.isBusy());
    }

    @Test
    void retargetingASettledHoldRestartsBusyStateAndTimeout() throws InterruptedException {
        LocalizerStub localizer = new LocalizerStub();
        ForesightConfig config = config();
        Foresight algorithm = new Foresight(config);
        Follower follower = new Follower(localizer, new DriveStub(), algorithm);
        follower.hold(Pose.zero());
        follower.update(0.02);
        assertFalse(follower.isBusy());
        Thread.sleep(5);
        config.timeoutConstraint.set(0.0);
        assertTrue(algorithm.timeoutCondition());

        follower.hold(new Pose(10, 0));

        assertTrue(follower.isBusy());
        assertFalse(algorithm.timeoutCondition());
        config.timeoutConstraint.set(60000.0);
        follower.update(0.02);
        assertTrue(follower.isBusy());
    }

    @Test
    void enteringHoldResetsFeedbackControllers() {
        int[] resets = {0};
        ForesightConfig config = config();
        config.headingFeedback.set(new Controller() {
            @Override
            public double calculate(double target, double error) {
                return 0;
            }

            @Override
            public void reset() {
                resets[0]++;
            }
        });
        Follower follower = new Follower(new LocalizerStub(), new DriveStub(), new Foresight(config));

        follower.hold(new Pose(10, 0));
        follower.hold(new Pose(20, 0));

        assertEquals(2, resets[0]);
    }

    @Test
    void automaticEndpointHoldStillClearsBusyAfterSettling() {
        LocalizerStub localizer = new LocalizerStub();
        Follower follower = new Follower(localizer, new DriveStub(), new Foresight(config()));
        Pose target = new Pose(10, 0);
        follower.follow(line(Pose.zero(), target).constant(0));
        localizer.setPose(target);
        follower.update(0.02);
        follower.update(0.02);
        assertTrue(follower.holding());
        follower.update(0.02);
        assertFalse(follower.isBusy());
    }

    private static ForesightConfig config() {
        return new ForesightConfig(c -> {
            c.headingFeedback.set(Controller.proportional(0.1));
            c.forwardTranslational.set(Controller.proportional(0.1));
            c.strafeTranslational.set(Controller.proportional(0.1));
            c.brake.set(Controller.proportionalFeedforward(0.01));
            c.coast.set(Controller.proportional(0.01));
            c.linearBrakeCoefficients.set(new Matrix(new double[][] {{0.1, 0}, {0, 0.1}}));
            c.quadraticBrakeCoefficients.set(new Matrix(new double[][] {{0.01, 0}, {0, 0.01}}));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.01));
            c.maxAchievableForwardVelocity.set(100.0);
            c.maxAchievableStrafeVelocity.set(100.0);
            c.naturalForwardDeceleration.set(100.0);
            c.naturalStrafeDeceleration.set(100.0);
            c.timeoutConstraint.set(60000.0);
        });
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
}
