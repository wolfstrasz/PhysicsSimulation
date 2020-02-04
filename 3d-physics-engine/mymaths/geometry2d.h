#ifndef _H_MYMATHS_GEOMETRY2D
#define _H_MYMATHS_GEOMETRY2D

#include "vectors.h"

// Name conventions
#define PointLine(point, line) PointOnLine(point, line)
#define LinePoint(line, point) PointOnLine(point, line)
#define CircleLine(circle, line) LineCircle(line, circle)
#define RectangleLine(rectangle, line) LineRectangle(line, rectangle);
#define OrientedRectangleLine(rectangle, line) \
  LineOrientedRectangle(line, rectangle);

// Points (basically vectors)
typedef vec2 Point2D;

// Lines
typedef struct Line2D {
  Point2D start;
  Point2D end;
  inline Line2D() {}
  inline Line2D(const Point2D& s, const Point2D& e) : start(s), end(e) {}
} Line2D;

// Line Func
float Length(const Line2D& line);
float LengthSq(const Line2D& line);

// Circles
typedef struct Circle {
  Point2D position;
  float radius;
  inline Circle() : radius(1.0f) {}
  inline Circle(const Point2D& p, float r) : position(p), radius(r) {}

} Circle;

// Rectangles
typedef struct Rectangle2D {
  Point2D origin;
  vec2 size;
  inline Rectangle2D() : size(1, 1) {}
  inline Rectangle2D(const Point2D& o, const vec2& s) : origin(o), size(s) {}
} Rectangle2D;

// Rectangle Func
vec2 GetMin(const Rectangle2D& rect);
vec2 GetMax(const Rectangle2D& rect);

Rectangle2D FromMinMax(const vec2& min, const vec2& max);

// Oriented Rectangle
typedef struct OrientedRectangle {
  Point2D position;
  vec2 halfExtents;
  float rotation;

  inline OrientedRectangle() : halfExtents(1.0f, 1.0f), rotation(0.0f) {}
  inline OrientedRectangle(const Point2D& p, const vec2& e)
      : position(p), halfExtents(e), rotation(0.0f) {}
  inline OrientedRectangle(const Point2D& pos, const vec2& ext, float rot)
      : position(pos), halfExtents(ext), rotation(rot) {}
} OrientedRectangle;

// Interval of points (for projection on line)
typedef struct Interval2D {
  float min;
  float max;
} Interval2D;

// Interval Functionality
Interval2D GetInterval(const Rectangle2D& rect, const vec2& axis);
Interval2D GetInterval(const OrientedRectangle& rect, const vec2& axis);

// Point Containment Tests
bool PointOnLine(const Point2D& point, const Line2D& line);
bool PointInCircle(const Point2D& point, const Circle& circle);
bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle);
bool PointInOrientedRectangle(const Point2D& point,
                              const OrientedRectangle& rectangle);

// Line Intersection Tests
bool LineCircle(const Line2D& line, const Circle& circle);
bool LineRectangle(const Line2D& line, const Rectangle2D& rectangle);
bool LineOrientedRectangle(const Line2D& line,
                           const OrientedRectangle& rectangle);

// Collision Test :: Circle and Circle
bool CircleCircle(const Circle& c1, const Circle& c2);

// Collision Test :: Circle and Rectangle
bool CircleRectangle(const Circle& circle, const Rectangle2D& rectangle);
#define RectangleCircle(rectangle, circle) CircleRectangle(circle, rectangle)

// Collision Test :: Circle and Oriented Rectangle
bool CircleOrientedRectangle(const Circle& circle,
                             const OrientedRectangle& rect);
#define OrientedRectangleCircle(rectangle, circle) \
  CircleOrientedRectangle(circle, rectangle)

// Collision Test :: Rectangle and Rectangle
bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2);

// Overlap on Axis Test
bool OverlapOnAxis(const Rectangle2D& rect1, const Rectangle2D& rect2,
                   const vec2& axis);
bool OverlapOnAxis(const Rectangle2D& rect1, const OrientedRectangle& rect2,
                   const vec2& axis);

// Separating Axis Theorem test
bool RectangleRectangleSAT(const Rectangle2D& rect1, const Rectangle2D& rect2);
bool RectangleOrientedRectangle(const Rectangle2D& rect1,
                                const OrientedRectangle& rect2);
#define OrientedRectangleRectangle(oriented, regular) \
  RectangleOrientedRectangle(regular, oriented)

bool OrientedRectangleOrientedRectangle(const OrientedRectangle& r1,
                                        const OrientedRectangle& r2);

// Additional 2D functionality
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

Circle ContainingCircle(Point2D* pArray, int arrayCount);
Rectangle2D ContainingRectangle(Point2D* pointArray, int arrayCount);

// Bounding Shape (does not own memory)
typedef struct BoundingShape {
  int numCircles;
  Circle* circles;
  int numRectangles;
  Rectangle2D* rectangles;
  inline BoundingShape()
      : numCircles(0), circles(0), numRectangles(0), rectangles(0) {}
} BoundingShape;

bool PointInShape(const BoundingShape& shape, const Point2D& point);

#endif  // !_H_MYMATHS_GEOMETRY2D
