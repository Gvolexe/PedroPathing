/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.localization;

import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector;
import com.pedropathing.utils.Angle;
import java.util.NavigableMap;
import java.util.TreeMap;

public class FusionLocalizer implements Localizer {
    private static class KalmanState {
        Pose pose;
        Twist twist;
        Pose relativeTransform;
        Matrix covariance;
        Pose remainderTransform;
        Long upperKey;

        public KalmanState(Pose pose, Twist twist, Pose relativeTransform, Matrix covariance) {
            this.pose = pose;
            this.twist = twist;
            this.relativeTransform = relativeTransform;
            this.covariance = covariance;
        }
    }

    public static double EPSILON = 1e-6; // floor for covariance matrices
    private final Localizer deadReckoning;
    private Pose currentRawPose;
    private MotionState motionState;
    private Pose currentRelativeTransform;
    private Matrix P; // State Covariance
    private final Matrix initialCovariance;
    private final Matrix Q; // Process Noise Covariance
    private final Matrix R; // Measurement Noise Covariance
    private long lastUpdateTime = -1;
    private final NavigableMap<Long, KalmanState> history = new TreeMap<>();
    private final int bufferSize;

    public FusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize) {
        this.deadReckoning = deadReckoning;
        motionState = MotionState.zero();
        currentRawPose = Pose.zero();

        // Standard Deviations for Kalman Filter
        this.P = Matrix.diag(
                Math.max(initialCovariance.x(), EPSILON),
                Math.max(initialCovariance.y(), EPSILON),
                Math.max(initialCovariance.heading(), EPSILON));
        this.initialCovariance = new Matrix(P.data, P.rows, P.cols);
        this.Q = Matrix.diag(
                Math.max(processVariance.x(), EPSILON),
                Math.max(processVariance.y(), EPSILON),
                Math.max(processVariance.heading(), EPSILON));
        this.R = Matrix.diag(
                Math.max(measurementVariance.x(), EPSILON),
                Math.max(measurementVariance.y(), EPSILON),
                Math.max(measurementVariance.heading(), EPSILON));
        this.bufferSize = bufferSize;
        history.put(0L, new KalmanState(Pose.zero(), Twist.zero(), currentRawPose, P));
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void update() {
        // Updates odometry
        deadReckoning.update();
        long now = System.nanoTime();
        double dt = lastUpdateTime < 0 ? 0 : (now - lastUpdateTime) / 1e9;
        lastUpdateTime = now;

        // Body-frame velocity remains valid when vision corrects the fused world frame.
        Twist currentTwist = deadReckoning.twist();

        // Update the pose estimate based on dead reckoning pose transformation
        Pose rawPose = deadReckoning.pose();
        currentRelativeTransform = currentRawPose.invert().compose(rawPose);

        // Update Kalman states
        P = updateCovariance(P, motionState.pose(), currentTwist, dt);
        Pose currentPosition = motionState.pose().compose(currentRelativeTransform);
        currentRawPose = rawPose;
        motionState = MotionState.ofTwist(currentPosition, currentTwist);

        history.put(now, new KalmanState(currentPosition, currentTwist, currentRelativeTransform, P));
        if (history.size() > bufferSize) history.pollFirstEntry();
    }

    /**
     * Resets odometry and the fused estimate to zero, discards vision history, and restores the initial covariance.
     * The wrapped pose is set explicitly because hardware resets may leave a stale cached state until the next update.
     */
    @Override
    public void reset() {
        deadReckoning.reset();
        deadReckoning.setPose(Pose.zero());
        motionState = MotionState.zero();
        currentRawPose = Pose.zero();
        currentRelativeTransform = Pose.zero();
        lastUpdateTime = -1;
        P = new Matrix(initialCovariance.data, initialCovariance.rows, initialCovariance.cols);
        history.clear();
        history.put(System.nanoTime(), new KalmanState(Pose.zero(), Twist.zero(), Pose.zero(), P));
    }

    /**
     * Consider the system xₖ₊₁ = xₖ + (f(xₖ, uₖ) + wₖ) * Δt.
     * <p>
     * wₖ is the noise in the system caused by sensor uncertainty, a zero-mean random vector with covariance Q.
     * <p>
     * The Kalman Filter update step is given by:
     * <pre>
     *     Pₖ₊₁ = F * Pₖ * Fᵀ + G * Q * Gᵀ
     * </pre>
     * Here F and G represent the State Transition Matrix and Control-to-State Matrix respectively.
     * <p>
     * The State Transition Matrix F is given by I + ∂f/∂x.
     * We computed our twist integration using a first-order forward-Euler approximation.
     * Therefore, f only depends on the twist, not on x, so ∂f/∂x = 0 and F = I.
     * <p>
     * The Control-to-State Matrix G is given by ∂xₖ₊₁ / ∂wₖ.
     * Here this is simply I * Δt.
     * <p>
     * The Kalman update is Pₖ₊₁ = F * Pₖ * Fᵀ + G * Q * Gᵀ.
     * With F = I and G = I * Δt, we get Pₖ₊₁ = Q * Δt².
     *
     * @param dt the time step Δt in seconds
     */
    private Matrix updateCovariance(Matrix P, Pose pose, Twist twist, double dt) {
        Vector dist = twist.toVector().abs().times(dt);
        Vector Q_diag = new Vector(Q.getDiagonal());
        Matrix bodyQ = Matrix.diag(dist.hadamard(Q_diag));
        Matrix rotation = Matrix.rotationTransform(pose.heading());
        Matrix worldQ = rotation.times(bodyQ).times(rotation.transpose());
        P = P.plus(worldQ).clampDiagonals(EPSILON);
        return P;
    }

    private KalmanState getKalmanState() {
        return new KalmanState(motionState.pose(), motionState.twist(), currentRelativeTransform, P);
    }

    /**
     * Adds a vision measurement using the default measurement variance
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     */
    public void addMeasurement(Pose measuredPose, long timestamp) {
        addMeasurement(measuredPose, timestamp, null);
    }

    /**
     * Adds a vision measurement with a custom variance for this specific measurement
     * @param measuredPose the measured position by the camera, enter NaN to a specific axis if the camera couldn't measure that axis
     * @param timestamp the timestamp of the measurement
     * @param measurementVariance the variance for this specific measurement (x, y, heading), or null to use the default
     */
    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        Matrix measurementR = measurementVariance == null
                ? R
                : Matrix.diag(measurementVariance.x(), measurementVariance.y(), measurementVariance.heading())
                        .clampDiagonals(EPSILON);

        // Reject if timestamp is outside our poseHistory time window
        if (history.isEmpty() || timestamp < history.firstKey() || timestamp > history.lastKey()) return;

        KalmanState interpolatedData = interpolate(timestamp);
        if (interpolatedData == null) interpolatedData = getKalmanState();
        Pose pastPose = interpolatedData.pose;

        if (pastPose == null) pastPose = pose();

        // Measurement residual y = z - x
        boolean measX = !Double.isNaN(measuredPose.x());
        boolean measY = !Double.isNaN(measuredPose.y());
        boolean measH = !Double.isNaN(measuredPose.heading());

        Vector innovation = new Vector(
                measX ? measuredPose.x() - pastPose.x() : 0,
                measY ? measuredPose.y() - pastPose.y() : 0,
                measH ? Angle.normalizeSigned(measuredPose.heading() - pastPose.heading()) : 0);
        // Measurement mask M
        Matrix M = Matrix.diag(measX ? 1 : 0, measY ? 1 : 0, measH ? 1 : 0);

        // Covariance at measurement time
        Matrix Pm = interpolatedData.covariance;

        // Unmeasured axes are absent observations, not measurements of zero innovation.
        Matrix S = M.times(Pm).times(M.transpose()).plus(measurementR);

        // Apply gain K = P * H^T * (H * P * H^T + R)^(-1), with H = M.
        Matrix S_inv = S.invert();
        if (S_inv == null) return;
        Matrix K = Pm.times(M.transpose()).times(S_inv);

        if (interpolatedData.upperKey != null) {
            KalmanState upperEntry = history.get(interpolatedData.upperKey);
            history.put(
                    interpolatedData.upperKey,
                    new KalmanState(
                            upperEntry.pose,
                            upperEntry.twist,
                            interpolatedData.remainderTransform,
                            upperEntry.covariance));
        }

        innovation = M.times(innovation);

        // State update
        Vector Ky = K.times(innovation);
        Pose updatedPast = new Pose(
                pastPose.x() + Ky.get(0), pastPose.y() + Ky.get(1), Angle.normalize(pastPose.heading() + Ky.get(2)));

        // Joseph-form covariance update
        Matrix I = Matrix.identity(3);
        Matrix IK = I.minus(K.times(M));
        Matrix cov = IK.times(Pm)
                .times(IK.transpose())
                .plus(K.times(measurementR).times(K.transpose()))
                .clampDiagonals(EPSILON);
        history.put(
                timestamp,
                new KalmanState(updatedPast, interpolatedData.twist, interpolatedData.relativeTransform, cov));

        // Forward propagate pose + covariance
        long prevTime = timestamp;
        Pose prevPose = updatedPast;

        for (NavigableMap.Entry<Long, KalmanState> entry :
                history.tailMap(timestamp, false).entrySet()) {
            long t = entry.getKey();
            Twist twist = entry.getValue().twist;
            if (twist == null) twist = twist();

            double dt = (t - prevTime) / 1e9;

            Pose relativeTransform = entry.getValue().relativeTransform;
            cov = updateCovariance(cov, prevPose, twist, dt);
            prevPose = prevPose.compose(relativeTransform);
            history.put(t, new KalmanState(prevPose, twist, entry.getValue().relativeTransform, cov));
            prevTime = t;
        }

        KalmanState latest = history.lastEntry().getValue();
        motionState = MotionState.ofTwist(latest.pose, latest.twist);
        P = history.lastEntry().getValue().covariance;
    }

    private KalmanState interpolate(long timestamp) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey);

        KalmanState lower = history.get(lowerKey);
        KalmanState upper = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (upperKey - lowerKey);

        Twist lowerTwist = lower.twist;
        Twist upperTwist = upper.twist;
        Twist interpolTwist = lowerTwist.plus(upperTwist.minus(lowerTwist).times(ratio));
        Pose toMeasurement = interpolateTransform(Pose.zero(), upper.relativeTransform, ratio);
        // Split the historical pose with the same SE(2) interpolation as its odometry transform.
        // With zero innovation, composing this pose and the remainder must recover the upper sample.
        Pose interpolPose = lower.pose.compose(toMeasurement);
        Pose remainder = toMeasurement.invert().compose(upper.relativeTransform);

        KalmanState result = new KalmanState(interpolPose, interpolTwist, toMeasurement, lower.covariance);
        result.remainderTransform = remainder;
        result.upperKey = upperKey;
        return result;
    }

    public static Pose interpolateTransform(Pose a, Pose b, double ratio) {
        // Linear interpolation in twist space
        Twist delta = Twist.riemannianLog(a, b).times(ratio);
        return a.compose(Pose.zero().exp(delta));
    }

    public void setStartPose(Pose setStart) {
        deadReckoning.setPose(setStart);
        history.clear();
        history.put(0L, new KalmanState(setStart, Twist.zero(), setStart, P));
        motionState = MotionState.ofTwist(setStart, motionState.twist());
        currentRawPose = setStart;
    }

    /**
     * Teleports the estimate and discards filter history. Use at init or startup to hard set a robot's location or to hard reset to a known location during a match.
     */
    @Override
    public void setPose(Pose setPose) {
        motionState = MotionState.ofTwist(setPose, motionState.twist());
        deadReckoning.setPose(setPose);
        currentRawPose = setPose;

        history.clear();
        history.put(System.nanoTime(), new KalmanState(setPose, motionState.twist(), Pose.zero(), P));
    }

    /**
     * Applies a small non-timestamped correction while preserving filter history. For vision, use timestamped addMeasurement.
     */
    public void correctPose(Pose correctedPose) {
        Pose shift = correctedPose.compose(motionState.pose().invert());
        for (KalmanState state : history.values()) {
            state.pose = shift.compose(state.pose);
        }

        motionState = MotionState.ofTwist(correctedPose, motionState.twist());
        deadReckoning.setPose(correctedPose);
        currentRawPose = correctedPose;
    }
}
