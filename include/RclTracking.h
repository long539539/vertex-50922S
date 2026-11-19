#pragma once

// RCL Tracking for mikLib (VEXcode), ported from https://github.com/jiazegao/RCL-Tracking (LemLib / PROS).
//
// Continuous, automatic distance resets in the background. Every update, each distance sensor whose beam is
// close to square with a field wall gives the robot's X or Y position. RCL averages them, then gently pulls
// odometry toward that position (at most maxSyncPerSec inches per second) so motions stay smooth.
// Beams that would hit a declared obstacle (a goal, a field element) are ignored.
//
// Coordinates are real field coordinates in inches: field center (0, 0), walls at +-70.25, heading 0 = +Y,
// clockwise positive, same as mikLib odometry. Set it up in src/rcl-config.cpp.

#include "v5.h"
#include "v5_vcs.h"
#include <cstdint>
#include <utility>
#include <vector>

class Chassis;

// Consts
constexpr double mmToInch = 0.039370078740157;
constexpr double FIELD_HALF_LENGTH = 70.25;        // inches. Matches mikLib's reset_axis (original RCL uses 70.5)
constexpr double FIELD_NEG_HALF_LENGTH = -70.25;

// Timer (milliseconds), used for obstacle lifetimes and loop timing
class Timer {
public:
    Timer(double timeoutMs_ = 0) : timeoutMs(timeoutMs_), startMs(vex::timer::system()) {}
    void reset() { startMs = vex::timer::system(); }
    void hardReset(double newTimeoutMs) { timeoutMs = newTimeoutMs; reset(); }
    bool timeIsUp() const { return elapsed() > timeoutMs; }
    double timeLeft() const { double e = elapsed(); return e < timeoutMs ? (timeoutMs - e) : 0; }
    double elapsed() const { return static_cast<double>(vex::timer::system() - startMs); }

private:
    double timeoutMs;
    uint32_t startMs;
};

// Robot position: x, y in inches, theta in degrees (0 = +Y, clockwise positive)
struct RclPose {
    double x = 0;
    double y = 0;
    double theta = 0;
};

enum class CoordType { X, Y, INVALID };

// Pose of a distance sensor's beam
struct SensorPose {
    double x = 0;
    double y = 0;
    double heading = 0;
    double slope = 0;
    double yIntercept = 0;
};

struct Line {
    double pt1[2] = {0, 0};
    double pt2[2] = {0, 0};
};

/**
 * A line segment on the field. Sensor beams that cross it are ignored.
 * Ex: Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH); ignores the far side of the field.
 */
class Line_Obstacle {
public:
    /** @param lifeTimeMs How long the obstacle lasts after it's created. Negative = forever. */
    Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs = -1);
    bool expired() const;
    bool isIntersecting(const SensorPose& sp) const;
    /** @brief Adds a closed polygon made of line obstacles. Points are {x, y} pairs, at least 3. */
    static void addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs = -1);
    static std::vector<Line_Obstacle*>& obstacleCollection();

private:
    Line line;
    Timer lifeTimer;
};

/** A circle on the field, ex: a goal. Sensor beams that pass through it are ignored. */
class Circle_Obstacle {
public:
    /** @param lifeTimeMs How long the obstacle lasts after it's created. Negative = forever. */
    Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs = -1);
    bool expired() const;
    bool isIntersecting(const SensorPose& sp) const;
    static std::vector<Circle_Obstacle*>& obstacleCollection();

    double x, y, radius;
    Timer lifeTimer;
};

/** A distance sensor used by RCL. */
class RclSensor {
public:
    /**
     * @param distSensor The distance sensor.
     * @param horizOffset Inches right of the tracking center (left is negative).
     * @param vertOffset Inches in front of the tracking center (behind is negative).
     * @param mainAng Which way the sensor faces: 0 = front, 90 = right, 180 = back, 270 = left.
     * @param angleTol The sensor is only used while its beam is within this many degrees of square to a wall.
     */
    RclSensor(vex::distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol = 15.0);

    void updatePose(const RclPose& botPose);
    bool isValid(double distValMm) const;
    /** @return Which coordinate this sensor measures right now (X, Y or INVALID) and its value in inches.
     *  @param accumMm An averaged reading to use instead of a live one, in mm. Negative = use a live reading. */
    std::pair<CoordType, double> getBotCoord(const RclPose& botPose, double accumMm = -1);
    /** @return The live reading in mm (about 9999 when nothing is detected). */
    double rawReading() const;
    SensorPose getPose() const;

    static std::vector<RclSensor*>& sensorCollection();

private:
    vex::distance* sensor;
    double offsetDist;
    double offsetAngle;
    double mainAngle;
    SensorPose sp;
    double angleTolerance;  // degrees
};

class RclTracking {
public:
    /**
     * @param chassis_ The mikLib chassis (&chassis).
     * @param frequencyHz_ Updates per second.
     * @param autoSync_ Automatically pull odometry toward the RCL position.
     * @param minDelta_ A sensor-based position must differ from the current one by at least this (in) to be used.
     *                  Ignores normal sensor jitter.
     * @param maxDelta_ A sensor-based position more than this (in) from the current one is ignored. Ex: a robot in the way.
     * @param maxDeltaFromOdom_ Fail safe: how far (in) the RCL position may drift from odometry.
     * @param maxSyncPerSec_ The fastest odometry is pulled toward the RCL position (in/s).
     * @param minPause_ Minimum milliseconds each loop sleeps.
     * @param maxRange_ Readings farther than this (in) are ignored. 0 = no limit. Not in the original RCL: it stands in
     *                  for PROS's confidence check, which VEXcode doesn't have. Long readings are the noisiest.
     * @param smoothingSamples_ Use the median of the last few corrections (within 400 ms) instead of only the newest.
     *                  0 = off. Not in the original RCL. Helps when each axis is seen by only one sensor.
     */
    RclTracking(Chassis* chassis_,
                int frequencyHz_ = 25,
                bool autoSync_ = true,
                double minDelta_ = 0.5,
                double maxDelta_ = 4.0,
                double maxDeltaFromOdom_ = 10.0,
                double maxSyncPerSec_ = 3.0,
                int minPause_ = 20,
                double maxRange_ = 0,
                int smoothingSamples_ = 0);

    /** @brief Starts the background loops. Call it after chassis.set_coordinates(). */
    void startTracking();
    /** @brief Stops the background loops. */
    void stopTracking();

    /** @return The robot position calculated by RCL. */
    RclPose getRclPose() const;
    /** @brief Sets the RCL position. */
    void setRclPose(const RclPose& p);
    /** @brief Snaps odometry to the RCL position right away. Not needed while autoSync is on. */
    void updateBotPose();
    /** @brief Standard distance reset from one sensor, with every filter off except invalid readings.
     *  Only use it when nothing can be between that sensor and the wall. */
    void updateBotPose(RclSensor* sens);

    void setMaxSyncPerSec(double maxSyncPerSec_);
    /** @brief Readings farther than this (in) are ignored. 0 = no limit. */
    void setMaxRange(double inches);
    /** @brief Median of the last few corrections instead of only the newest. 0 = off. */
    void setSmoothing(int samples);

    /** @brief While the robot sits still, average each sensor's readings for more accuracy. */
    void startAccumulating(bool autoUpdateAfterAccum = true);
    /** @brief Stops averaging and uses the averages. */
    void stopAccumulating();
    /** @brief Averages readings for ms milliseconds, then uses them. Blocks until done. */
    void accumulateFor(int ms, bool autoUpdateAfterAccum = true);

    /** @brief Forgets RCL's corrections and starts again from odometry. Call it after chassis.set_coordinates() or
     *  chassis.reset_axis() while RCL is running. Big odometry jumps are also detected automatically. */
    void discardData();

    // Single updates
    void mainUpdate();
    void syncUpdate();

private:
    Chassis* chassis;
    int goalMSPT;
    int minPause;
    double maxSyncPT;
    double minDelta, maxDelta, maxDeltaFromOdom;
    bool autoSync;
    volatile bool accumulating = false;
    volatile bool updateAfterAccum = false;
    bool running = false;
    vex::task mainLoopTask;
    vex::task miscLoopTask;
    RclPose latestPrecise, poseAtLatest;

    double lastOdomX = 0, lastOdomY = 0;
    uint32_t lastOdomMs = 0;

    double maxRangeMm = 0;
    int smoothingSamples = 0;
    // Recent corrections, per axis (0 = X, 1 = Y), as {time ms, sensor position minus unsynced odometry}
    std::vector<std::pair<uint32_t, double>> history[2];
    // Total distance RCL itself has moved odometry, so its own nudges aren't mistaken for driving
    double ownShiftX = 0, ownShiftY = 0;

    RclPose odomPose() const;
    void setOdomXY(double x, double y);
    void watchForOdomResets();
    bool smoothedValue(int axis, double value, double odomAxis, uint32_t now, double& out);

    static int mainLoopEntry(void* self);
    static int miscLoopEntry(void* self);
    void mainLoop();
    void miscLoop();
};
