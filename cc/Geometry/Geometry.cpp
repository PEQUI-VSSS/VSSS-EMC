#include "Geometry.h"

using namespace Geometry;

double Geometry::wrap(double theta) {
	theta = std::fmod(theta, 2 * M_PI);
	if (theta > M_PI) return theta - 2 * M_PI;
	else if (theta < -M_PI) return theta + 2 * M_PI;
	else return theta;
}

double Geometry::distance(const Point &a, const Point &b) {
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double Geometry::distance_x(const Point &a, const Point &b) {
	return std::abs(a.x - b.x);
}

double Geometry::distance_y(const Point &a, const Point &b) {
	return std::abs(a.y - b.y);
}

Point Geometry::intersection(const Line &l1, const Line &l2) {
	if(l1.a == l2.a && l1.b == l2.b) return {0, l1.b};
	else {
		double x = (l2.b - l1.b)/(l1.a - l2.a);
		return {x, l1.a * x + l1.b};
	};
}

Vector Point::operator-(const Point &p2) const {
	return {distance(*this, p2), std::atan2(y - p2.y, x - p2.x)};
}

Point Point::operator+(const Vector &v) const {
	return {x + v.size * std::cos(v.theta),
			y + v.size * std::sin(v.theta)};
}

Point Point::operator-(const Vector &v) const {
	return {x - v.size * std::cos(v.theta),
			y - v.size * std::sin(v.theta)};
}

Point Geometry::from_cv_point(const cv::Point cv_point) {
	return from_cv_point(cv_point.x, cv_point.y);
}

Point Geometry::from_cv_point(const double x, const double y) {
	return {x * (1.7 / 640.0),
			1.3 - y * (1.3 / 480.0)};
}

cv::Point Point::to_cv_point() {
	return {static_cast<int>(x * 640 / 1.7),
			480 - static_cast<int>(y * 480 / 1.3) };
}

Vector::Vector(const Point &p) {
	size = distance(p, {0,0});
	theta = std::atan2(p.y, p.x);
}

Vector Vector::unitary() {
	return {1, theta};
}

Vector Vector::with_size(double new_size) {
	return {new_size, theta};
}

Vector Vector::operator*(double value) {
	return {size * value, theta};
}

bool Rectangle::intersect(const Rectangle &b) {
	auto& a = *this;
	for(int polyi = 0; polyi < 2; ++polyi) {

		const Rectangle& polygon = (polyi == 0) ? a : b;

		for(int i1 = 0; i1 < (int) polygon.points.size(); ++i1) {
			const int i2 = (i1 + 1) % (int) polygon.points.size();

			const double normalx = polygon.points[i2].y - polygon.points[i1].y;
			const double normaly = polygon.points[i2].x - polygon.points[i1].x;

			double minA = std::numeric_limits<double>::max();
			double maxA = std::numeric_limits<double>::min();

			for(auto & point : a.points) {
				const double projected = normalx * point.x +
										 normaly * point.y;
				if(projected < minA) minA = projected;
				if(projected > maxA) maxA = projected;
			}

			double minB = std::numeric_limits<double>::max();
			double maxB = std::numeric_limits<double>::min();

			for(auto point : b.points) {
				const double projected = normalx * point.x +
										 normaly * point.y;
				if( projected < minB ) minB = projected;
				if( projected > maxB ) maxB = projected;
			}

			if(maxA < minB || maxB < minA)
				return false;
		}
	}

	return true;
}

#include <Robot2.h>
static constexpr double HALF_ROBOT = Robot2::SIZE/2;
Rectangle Rectangle::from_robot(Point p, Vector to_front, double offset) {
	auto to_left = Vector(to_front.theta + degree_to_rad(90), HALF_ROBOT);
	auto to_right = Vector(to_front.theta + degree_to_rad(-90), HALF_ROBOT);
	auto middle_left = p + to_left;
	auto front_left = middle_left + to_front.with_size(HALF_ROBOT + offset);
	auto back_left = middle_left - to_front.with_size(HALF_ROBOT + offset);
	auto middle_right = p + to_right;
	auto front_right = middle_right + to_front.with_size(HALF_ROBOT + offset);
	auto back_right = middle_right - to_front.with_size(HALF_ROBOT + offset);
	return Rectangle{{back_left, front_left, front_right, back_right}};
}
