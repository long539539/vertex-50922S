// RclTracking.cpp
// RCL Tracking for mikLib (VEXcode), ported from https://github.com/jiazegao/RCL-Tracking (LemLib / PROS).
// Sensor management, obstacle detection, position tracking and syncing using V5 distance sensors.
// Most of it is basic sensor fusion and intersection math.

#include "vex.h"
#include "RclTracking.h"
#include <algorithm>
#include <numeric>

namespace {
constexpr double kPi = 3.14159265358979323846;

// Smart ports are indexes 0-20. mikLib's PORT0 (nothing plugged in) is far outside that.
constexpr int32_t kLastSmartPortIndex = 20;

// Odometry jumping farther or faster than the robot can drive means set_coordinates() moved it.
constexpr double kResetJump = 12.0;    // inches in one update
constexpr double kResetSpeed = 100.0;  // in/s. This drivetrain tops out near 65 in/s.

// Compass heading (0 = +Y, clockwise) to a math angle (0 = +X, counter-clockwise), degrees
double botToTrig(double ang) {
    double result = 90.0 - ang;
    while (result > 360.0) result -= 360.0;
    while (result < 0) result += 360.0;
    return result;
}

double degToRad(double deg) { return deg * kPi / 180.0; }

// mikLib keeps odometry private, and its only public setter, set_coordinates(), also resets the tracking
// wheels, the inertial heading and the odometry task, which would break a motion in progress. RCL nudges the
// position continuously the way LemLib's setPose() does, so this reaches Chassis::odom without editing mikLib.
// Explicit template instantiation is allowed to name private members (C++ standard, [temp.explicit]).
// If a mikLib update ever renames that member, this stops compiling instead of failing silently.
struct ChassisOdomTag {
    using type = ::odom Chassis::*;
    friend type chassisOdomMember(ChassisOdomTag);
};
template <typename Tag, typename Tag::type Member>
struct PrivateMemberAccess {
    friend typename Tag::type chassisOdomMember(Tag) { return Member; }
};
template struct PrivateMemberAccess<ChassisOdomTag, &Chassis::odom>;
}  // namespace

// ---------------------------------------------------------------------------------------------------------
// Line obstacles
// ---------------------------------------------------------------------------------------------------------

std::vector<Line_Obstacle*>& Line_Obstacle::obstacleCollection() {
    static std::vector<Line_Obstacle*> collection;
    return collection;
}

Line_Obstacle::Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs)
    : lifeTimer(lifeTimeMs < 0 ? 1e12 : lifeTimeMs) {
    line.pt1[0] = x1;
    line.pt1[1] = y1;
    line.pt2[0] = x2;
    line.pt2[1] = y2;
    obstacleCollection().push_back(this);
}

bool Line_Obstacle::expired() const { return lifeTimer.timeIsUp(); }

bool Line_Obstacle::isIntersecting(const SensorPose& sp) const {
    // Ray direction vector
    const double angRad = degToRad(botToTrig(sp.heading));
    const double vAx = cos(angRad);
    const double vAy = sin(angRad);

    // Obstacle segment vector
    const double vBx = line.pt2[0] - line.pt1[0];
    const double vBy = line.pt2[1] - line.pt1[1];

    // If the determinant is 0, the ray and the obstacle are parallel
    const double det = (vAx * vBy) - (vAy * vBx);
    if (fabs(det) < 1e-6) return false;

    // Solve P_sensor + t*vA = P_obstacle_start + u*vB
    const double dx = line.pt1[0] - sp.x;
    const double dy = line.pt1[1] - sp.y;
    const double t = (dx * vBy - dy * vBx) / det;
    const double u = (dx * vAy - dy * vAx) / det;

    // In front of the sensor, and on the segment itself
    return (t > 0 && u >= 0 && u <= 1);
}

void Line_Obstacle::addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs) {
    if (points.size() < 3) return;
    for (size_t i = 0; i < points.size(); ++i) {
        const size_t next = (i + 1) % points.size();
        new Line_Obstacle(points[i].first, points[i].second, points[next].first, points[next].second, lifeTimeMs);
    }
}

// ---------------------------------------------------------------------------------------------------------
// Circle obstacles
// ---------------------------------------------------------------------------------------------------------

std::vector<Circle_Obstacle*>& Circle_Obstacle::obstacleCollection() {
    static std::vector<Circle_Obstacle*> collection;
    return collection;
}

Circle_Obstacle::Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs)
    : x(x_), y(y_), radius(r_), lifeTimer(lifeTimeMs < 0 ? 1e12 : lifeTimeMs) {
    obstacleCollection().push_back(this);
}

bool Circle_Obstacle::expired() const { return lifeTimer.timeIsUp(); }

bool Circle_Obstacle::isIntersecting(const SensorPose& sp) const {
    const double angRad = degToRad(botToTrig(sp.heading));
    const double vx = cos(angRad);
    const double vy = sin(angRad);

    // Distance along the ray to the point closest to the circle's center
    const double dx = x - sp.x;
    const double dy = y - sp.y;
    const double t = dx * vx + dy * vy;
    if (t < 0) return false;  // behind the sensor

    const double closestX = sp.x + t * vx;
    const double closestY = sp.y + t * vy;
    const double distSq = (closestX - x) * (closestX - x) + (closestY - y) * (closestY - y);
    return distSq <= (radius * radius);
}

// ---------------------------------------------------------------------------------------------------------
// RCL sensor
// ---------------------------------------------------------------------------------------------------------

std::vector<RclSensor*>& RclSensor::sensorCollection() {
    static std::vector<RclSensor*> collection;
    return collection;
}

RclSensor::RclSensor(vex::distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol)
    : sensor(distSensor), mainAngle(mainAng), angleTolerance(fabs(angleTol)) {
    offsetDist = hypot(horizOffset, vertOffset);
    offsetAngle = fmod((atan2(vertOffset, horizOffset) * 180.0 / kPi) + 360.0, 360.0);
    sensorCollection().push_back(this);
}

// Where the sensor is on the field and which way its beam points
void RclSensor::updatePose(const RclPose& botPose) {
    const double theta = degToRad(offsetAngle - botPose.theta);
    sp.x = botPose.x + cos(theta) * offsetDist;
    sp.y = botPose.y + sin(theta) * offsetDist;
    sp.heading = fmod(botPose.theta + mainAngle, 360.0);
    if (sp.heading <= 0) sp.heading += 360.0;
    sp.slope = tan(degToRad(botToTrig(sp.heading)));
    sp.yIntercept = sp.y - sp.slope * sp.x;
}

bool RclSensor::isValid(double distValMm) const {
    if (sensor == nullptr) return false;
    const int32_t port = sensor->index();
    if (port < 0 || port > kLastSmartPortIndex || !sensor->installed()) return false;  // PORT0 or unplugged
    if (distValMm <= 0 || distValMm > 2000) return false;                                 // invalid distance
    // PROS's get_confidence() has no VEXcode equivalent, so require that the sensor detects something.
    if (!sensor->isObjectDetected()) return false;
    // Only use beams close to square with the walls
    const double offSquare = fabs(fmod(sp.heading, 90.0));
    if (offSquare > angleTolerance && offSquare < (90.0 - angleTolerance)) return false;
    // Skip beams that pass through an obstacle
    for (auto* item : Circle_Obstacle::obstacleCollection()) {
        if (!item->expired() && item->isIntersecting(sp)) return false;
    }
    for (auto* item : Line_Obstacle::obstacleCollection()) {
        if (!item->expired() && item->isIntersecting(sp)) return false;
    }
    return true;
}

std::pair<CoordType, double> RclSensor::getBotCoord(const RclPose& botPose, double accumMm) {
    updatePose(botPose);

    double val = accumMm >= 0 ? accumMm : rawReading();
    if (!isValid(val)) return {CoordType::INVALID, 0.0};
    val *= mmToInch;

    const double angRad = degToRad(botToTrig(sp.heading));
    const double cosA = cos(angRad);
    const double sinA = sin(angRad);

    // Which wall the beam hits first
    double minDist = 1e9;
    int wall = -1;
    if (fabs(cosA) > 1e-6) {
        const double dEast = (FIELD_HALF_LENGTH - sp.x) / cosA;
        if (dEast > 0 && dEast < minDist) { minDist = dEast; wall = 2; }
        const double dWest = (FIELD_NEG_HALF_LENGTH - sp.x) / cosA;
        if (dWest > 0 && dWest < minDist) { minDist = dWest; wall = 4; }
    }
    if (fabs(sinA) > 1e-6) {
        const double dNorth = (FIELD_HALF_LENGTH - sp.y) / sinA;
        if (dNorth > 0 && dNorth < minDist) { minDist = dNorth; wall = 1; }
        const double dSouth = (FIELD_NEG_HALF_LENGTH - sp.y) / sinA;
        if (dSouth > 0 && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    // Sensor position from the wall and the reading
    double res;
    CoordType type;
    if (wall == 1)      { type = CoordType::Y; res = FIELD_HALF_LENGTH - sinA * val; }
    else if (wall == 2) { type = CoordType::X; res = FIELD_HALF_LENGTH - cosA * val; }
    else if (wall == 3) { type = CoordType::Y; res = FIELD_NEG_HALF_LENGTH - sinA * val; }
    else if (wall == 4) { type = CoordType::X; res = FIELD_NEG_HALF_LENGTH - cosA * val; }
    else return {CoordType::INVALID, 0.0};

    // Sensor position to tracking center
    const double offRad = degToRad(offsetAngle - botPose.theta);
    if (type == CoordType::X) res -= cos(offRad) * offsetDist;
    else res -= sin(offRad) * offsetDist;

    return {type, res};
}

double RclSensor::rawReading() const { return sensor->objectDistance(vex::mm); }
SensorPose RclSensor::getPose() const { return sp; }

// ---------------------------------------------------------------------------------------------------------
// RCL tracking
// ---------------------------------------------------------------------------------------------------------

RclTracking::RclTracking(Chassis* chassis_, int frequencyHz_, bool autoSync_, double minDelta_, double maxDelta_,
                         double maxDeltaFromOdom_, double maxSyncPerSec_, int minPause_, double maxRange_,
                         int smoothingSamples_)
    : chassis(chassis_),
      goalMSPT(static_cast<int>(round(1000.0 / frequencyHz_))),
      minPause(minPause_),
      maxSyncPT(maxSyncPerSec_ / frequencyHz_),
      minDelta(minDelta_),
      maxDelta(maxDelta_),
      maxDeltaFromOdom(maxDeltaFromOdom_),
      autoSync(autoSync_) {
    setMaxRange(maxRange_);
    setSmoothing(smoothingSamples_);
}

void RclTracking::startTracking() {
    if (running) return;
    discardData();
    lastOdomX = chassis->get_X_position();
    lastOdomY = chassis->get_Y_position();
    lastOdomMs = vex::timer::system();
    running = true;
    mainLoopTask = vex::task(mainLoopEntry, this);
    miscLoopTask = vex::task(miscLoopEntry, this);
}

void RclTracking::stopTracking() {
    if (!running) return;
    running = false;
    accumulating = false;
    mainLoopTask.stop();
    miscLoopTask.stop();
}

RclPose RclTracking::odomPose() const {
    return {chassis->get_X_position(), chassis->get_Y_position(), chassis->get_absolute_heading()};
}

void RclTracking::setOdomXY(double x, double y) {
    ::odom& od = chassis->*chassisOdomMember(ChassisOdomTag{});
    ownShiftX += x - od.position.x;
    ownShiftY += y - od.position.y;
    od.position.x = x;
    od.position.y = y;
}

RclPose RclTracking::getRclPose() const {
    const RclPose odom = odomPose();
    return {latestPrecise.x + (odom.x - poseAtLatest.x),
            latestPrecise.y + (odom.y - poseAtLatest.y),
            odom.theta};
}

void RclTracking::setRclPose(const RclPose& p) {
    latestPrecise = p;
    poseAtLatest = odomPose();
    history[0].clear();
    history[1].clear();
}

void RclTracking::updateBotPose() {
    const RclPose p = getRclPose();
    setOdomXY(p.x, p.y);
    setRclPose(p);
}

void RclTracking::updateBotPose(RclSensor* sens) {
    if (sens == nullptr || chassis == nullptr) return;
    const std::pair<CoordType, double> data = sens->getBotCoord(odomPose());
    RclPose pose = odomPose();
    if (data.first == CoordType::X) pose.x = data.second;
    else if (data.first == CoordType::Y) pose.y = data.second;
    else return;
    setOdomXY(pose.x, pose.y);
    setRclPose(pose);
}

void RclTracking::setMaxSyncPerSec(double maxSyncPerSec_) {
    maxSyncPT = maxSyncPerSec_ / (1000.0 / goalMSPT);
}

void RclTracking::setMaxRange(double inches) { maxRangeMm = inches > 0 ? inches / mmToInch : 0; }

void RclTracking::setSmoothing(int samples) {
    smoothingSamples = std::max(samples, 0);
    history[0].clear();
    history[1].clear();
}

// Adds a new correction for one axis and returns the median of the recent ones. The corrections are stored
// relative to odometry (minus RCL's own nudges), so the median doesn't lag behind while the robot drives.
bool RclTracking::smoothedValue(int axis, double value, double odomAxis, uint32_t now, double& out) {
    if (smoothingSamples <= 1) {
        out = value;
        return true;
    }
    const double reference = odomAxis - (axis == 0 ? ownShiftX : ownShiftY);
    std::vector<std::pair<uint32_t, double>>& h = history[axis];
    h.push_back({now, value - reference});
    h.erase(std::remove_if(h.begin(), h.end(),
                           [now](const std::pair<uint32_t, double>& e) { return now - e.first > 400; }),
            h.end());
    while (static_cast<int>(h.size()) > smoothingSamples) h.erase(h.begin());
    if (static_cast<int>(h.size()) < std::min(3, smoothingSamples)) return false;

    std::vector<double> offsets;
    for (const auto& e : h) offsets.push_back(e.second);
    std::sort(offsets.begin(), offsets.end());
    const size_t n = offsets.size();
    const double median = (n % 2) ? offsets[n / 2] : 0.5 * (offsets[n / 2 - 1] + offsets[n / 2]);
    out = reference + median;
    return true;
}

void RclTracking::startAccumulating(bool autoUpdateAfterAccum) {
    updateAfterAccum = autoUpdateAfterAccum;
    accumulating = true;
}

void RclTracking::stopAccumulating() { accumulating = false; }

void RclTracking::accumulateFor(int ms, bool autoUpdateAfterAccum) {
    startAccumulating(autoUpdateAfterAccum);
    Timer t(ms);
    while (!t.timeIsUp()) vex::task::sleep(minPause);
    stopAccumulating();
}

void RclTracking::discardData() {
    latestPrecise = odomPose();
    poseAtLatest = odomPose();
    history[0].clear();
    history[1].clear();
}

void RclTracking::mainUpdate() {
    std::vector<RclSensor*>& sensors = RclSensor::sensorCollection();
    if (sensors.empty()) return;

    // While accumulating, add up each sensor's valid readings
    std::vector<double> accTotal(sensors.size(), 0.0);
    std::vector<int> accCount(sensors.size(), 0);
    while (accumulating) {
        for (size_t i = 0; i < sensors.size(); i++) {
            const double reading = sensors[i]->rawReading();
            if (reading > 0 && reading <= 2000) {
                accTotal[i] += reading;
                accCount[i]++;
            }
        }
        vex::task::sleep(goalMSPT);
    }
    const bool usedAccum = std::any_of(accCount.begin(), accCount.end(), [](int c) { return c > 0; });

    // Keep the RCL position from straying too far from odometry
    RclPose botPose = getRclPose();
    const RclPose odom = odomPose();
    const double diffFromOdom = hypot(botPose.x - odom.x, botPose.y - odom.y);
    if (diffFromOdom > maxDeltaFromOdom) {
        botPose.x += (odom.x - botPose.x) / diffFromOdom;
        botPose.y += (odom.y - botPose.y) / diffFromOdom;
    }

    // Each sensor gives an X or a Y
    std::vector<double> xs, ys;
    for (size_t i = 0; i < sensors.size(); i++) {
        const double reading = accCount[i] > 0 ? accTotal[i] / accCount[i] : sensors[i]->rawReading();
        if (maxRangeMm > 0 && reading > maxRangeMm) continue;
        const std::pair<CoordType, double> result = sensors[i]->getBotCoord(botPose, reading);
        if (result.first == CoordType::INVALID) continue;
        const bool isX = result.first == CoordType::X;
        const double current = isX ? botPose.x : botPose.y;
        const double odomAxis = isX ? odom.x : odom.y;
        // Not in the original RCL: also accept readings that agree with odometry. Otherwise, once a few bad
        // readings (something in the beam) pull RCL more than maxDelta off, every good reading gets rejected
        // and RCL drags odometry away with it.
        if (fabs(result.second - current) <= maxDelta || fabs(result.second - odomAxis) <= maxDelta) {
            (isX ? xs : ys).push_back(result.second);
        }
    }

    // Use the averages. minDelta ignores normal jitter, but averaged readings skip it (they're already steady).
    const double minChange = usedAccum ? 0.0 : minDelta;
    const uint32_t now = vex::timer::system();
    if (!xs.empty()) {
        double meanX = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
        const bool ready = usedAccum || smoothedValue(0, meanX, odom.x, now, meanX);
        if (ready && meanX > FIELD_NEG_HALF_LENGTH && meanX < FIELD_HALF_LENGTH && fabs(meanX - botPose.x) >= minChange) {
            latestPrecise.x = meanX;
            poseAtLatest.x = chassis->get_X_position();
        }
    }
    if (!ys.empty()) {
        double meanY = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
        const bool ready = usedAccum || smoothedValue(1, meanY, odom.y, now, meanY);
        if (ready && meanY > FIELD_NEG_HALF_LENGTH && meanY < FIELD_HALF_LENGTH && fabs(meanY - botPose.y) >= minChange) {
            latestPrecise.y = meanY;
            poseAtLatest.y = chassis->get_Y_position();
        }
    }

    if (updateAfterAccum && usedAccum) updateBotPose();
}

void RclTracking::syncUpdate() {
    const RclPose rcl = getRclPose();
    const RclPose odom = odomPose();
    const double xDiff = rcl.x - odom.x;
    const double yDiff = rcl.y - odom.y;
    const double realDiff = hypot(xDiff, yDiff);

    if (realDiff <= maxSyncPT) {
        // Close enough: move odometry all the way
        setOdomXY(rcl.x, rcl.y);
        poseAtLatest.x += xDiff;
        poseAtLatest.y += yDiff;
    } else {
        // Otherwise move it maxSyncPT closer
        const double xUpdate = xDiff / realDiff * maxSyncPT;
        const double yUpdate = yDiff / realDiff * maxSyncPT;
        setOdomXY(odom.x + xUpdate, odom.y + yUpdate);
        poseAtLatest.x += xUpdate;
        poseAtLatest.y += yUpdate;
    }
    poseAtLatest.theta = odom.theta;
}

void RclTracking::watchForOdomResets() {
    const double x = chassis->get_X_position();
    const double y = chassis->get_Y_position();
    const uint32_t now = vex::timer::system();
    const double jump = hypot(x - lastOdomX, y - lastOdomY);
    const double dt = std::max(static_cast<double>(now - lastOdomMs) / 1000.0, 0.001);
    if (jump > kResetJump || (jump > 1.0 && jump / dt > kResetSpeed)) {
        discardData();  // set_coordinates() moved odometry; start over from the new position
    }
    lastOdomX = x;
    lastOdomY = y;
    lastOdomMs = now;
}

int RclTracking::mainLoopEntry(void* self) {
    static_cast<RclTracking*>(self)->mainLoop();
    return 0;
}

int RclTracking::miscLoopEntry(void* self) {
    static_cast<RclTracking*>(self)->miscLoop();
    return 0;
}

void RclTracking::mainLoop() {
    Timer frequencyTimer(goalMSPT);
    while (running) {
        frequencyTimer.reset();
        mainUpdate();
        // Sleep the rest of the period, but at least minPause
        vex::task::sleep(static_cast<uint32_t>(std::max(frequencyTimer.timeLeft(), static_cast<double>(minPause))));
    }
}

void RclTracking::miscLoop() {
    Timer frequencyTimer(goalMSPT);
    while (running) {
        frequencyTimer.reset();
        watchForOdomResets();
        if (autoSync) {
            syncUpdate();
            lastOdomX = chassis->get_X_position();  // our own sync isn't an outside reset
            lastOdomY = chassis->get_Y_position();
        }
        vex::task::sleep(static_cast<uint32_t>(std::max(frequencyTimer.timeLeft(), static_cast<double>(minPause))));
    }
}
