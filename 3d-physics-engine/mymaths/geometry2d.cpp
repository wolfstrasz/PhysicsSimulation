#include "Geometry2D.h"

#include <cfloat>
#include <cmath>

#include "matrices.h"

#define CMP(x, y) \
  (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

// Line Func
float Length(const Line2D& line) { return Magnitude(line.end - line.start); }
float LengthSq(const Line2D& line) {
  return MagnitudeSq(line.end - line.start);
}

// Rectangle Func
vec2 GetMin(const Rectangle2D& rect) {
  vec2 p1 = rect.origin;
  vec2 p2 = rect.origin + rect.size;
  return vec2(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}
vec2 GetMax(const Rectangle2D& rect) {
  vec2 p1 = rect.origin;
  vec2 p2 = rect.origin + rect.size;
  return vec2(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}
Rectangle2D FromMinMax(const vec2& min, const vec2& max) {
  return Rectangle2D(min, max - min);
}

// Point Containment Tests
bool PointOnLine(const Point2D& point, const Line2D& line) {
  // Find the slope
  float dy = (line.end.y - line.start.y);
  float dx = (line.end.x - line.start.x);
  float M = dy / dx;
  // Find the Y-Intercept
  float B = line.start.y - M * line.start.x;
  // Check line equation
  return CMP(point.y, M * point.x + B);
}

bool PointInCircle(const Point2D& point, const Circle& circle) {
  Line2D line(point, circle.position);
  if (LengthSq(line) < circle.radius * circle.radius) {
    return true;
  }
  return false;
}

bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle) {
  vec2 min = GetMin(rectangle);
  vec2 max = GetMax(rectangle);
  return min.x <= point.x && min.y <= point.y && point.x <= max.x &&
         point.y <= max.y;
}

bool PointInOrientedRectangle(const Point2D& point,
                              const OrientedRectangle& rectangle) {
  vec2 rotVector = point - rectangle.position;
  float theta = -DEG2RAD(rectangle.rotation);
  float zRotation2x2[] = {cosf(theta), sinf(theta), -sinf(theta), cosf(theta)};
  Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray, 1, 2,
           zRotation2x2, 2, 2);
  Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
  vec2 localPoint = rotVector + rectangle.halfExtents;
  return PointInRectangle(localPoint, localRectangle);
}

// Line Intersection Tests
bool LineCircle(const Line2D& line, const Circle& circle) {
  vec2 ab = line.end - line.start;
  float t = Dot(circle.position - line.start, ab) / Dot(ab, ab);
  if (t < 0.0f || t > 1.0f) {
    return false;
  }
  Point2D closestPoint = line.start + ab * t;
  Line2D circleToClosest(circle.position, closestPoint);
  return LengthSq(circleToClosest) < circle.radius * circle.radius;
}

bool LineRectangle(const Line2D& line, const Rectangle2D& rectangle) {
  if (PointInRectangle(line.start, rectangle) ||
      PointInRectangle(line.end, rectangle)) {
    return true;
  }
  vec2 norm = AsNormal(line.end - line.start);
  norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
  norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
  vec2 min = (GetMin(rectangle) - line.start) * norm;
  vec2 max = (GetMax(rectangle) - line.start) * norm;
  float tmin = fmaxf(fminf(min.x, max.x), fminf(min.y, max.y));
  float tmax = fminf(fmaxf(min.x, max.x), fmaxf(min.y, max.y));
  if (tmax < 0 || tmin > tmax) {
    return false;
  }
  float t = (tmin < 0.0f) ? tmax : tmin;
  return t > 0.0f && t * t < LengthSq(line);
}

bool LineOrientedRectangle(const Line2D& line,
                           const OrientedRectangle& rectangle) {
  float theta = -DEG2RAD(rectangle.rotation);
  float zRotation2x2[] = {cosf(theta), sinf(theta), -sinf(theta), cosf(theta)};
  Line2D localLine;
  vec2 rotVector = line.start - rectangle.position;
  Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray, 1, 2,
           zRotation2x2, 2, 2);
  localLine.start = rotVector + rectangle.halfExtents;
  rotVector = line.end - rectangle.position;
  Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray, 1, 2,
           zRotation2x2, 2, 2);
  localLine.end = rotVector + rectangle.halfExtents;
  Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
  return LineRectangle(localLine, localRectangle);
}

// Collision Test :: Circle and Circle
bool CircleCircle(const Circle& c1, const Circle& c2) {
  Line2D line(c1.position, c2.position);
  float radiiSum = c1.radius + c2.radius;
  return LengthSq(line) <= radiiSum * radiiSum;
}

// Collision Test :: Circle and Rectangle
bool CircleRectangle(const Circle& circle, const Rectangle2D& rect) {
  // Find min and max points
  vec2 min = GetMin(rect);
  vec2 max = GetMax(rect);

  // find the closest point on the rectangle to the position of the circle
  Point2D closestPoint = circle.position;

  closestPoint.x = (closestPoint.x < min.x) ? min.x : closestPoint.x;
  closestPoint.x = (closestPoint.x > max.x) ? max.x : closestPoint.x;

  closestPoint.y = (closestPoint.y < min.y) ? min.y : closestPoint.y;
  closestPoint.y = (closestPoint.y > max.y) ? max.y : closestPoint.y;

  // Check if point is inside circle
  Line2D line(circle.position, closestPoint);
  return LengthSq(line) <= circle.radius * circle.radius;
}

// Collision Test :: Circle and Oriented Rectangle
bool CircleOrientedRectangle(const Circle& circle,
                             const OrientedRectangle& rect) {
  // Create line between both centres
  vec2 r = circle.position - rect.position;

  // Construct rotation matrix
  float theta = -DEG2RAD(rect.rotation);
  float zRotation2x2[] = {cosf(theta), sinf(theta), -sinf(theta), cosf(theta)};

  // Rotate the centre of the circle
  Multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation2x2, 2, 2);

  // Create new local circle
  Circle lCircle(r + rect.halfExtents, circle.radius);

  // Create new local rectangle
  Rectangle2D lRect(Point2D(), rect.halfExtents * 2.0f);

  // Do circle-rectangle collision test
  return CircleRectangle(lCircle, lRect);
}

// Collision Test :: Rectangle and Rectangle

bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2) {
  // Get both min max
  vec2 aMin = GetMin(rect1);
  vec2 aMax = GetMax(rect1);
  vec2 bMin = GetMin(rect2);
  vec2 bMax = GetMax(rect2);
  // Check for overlap on both axes
  bool overX = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
  bool overY = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));

  return overX && overY;
}
