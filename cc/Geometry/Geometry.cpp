#include "Geometry.h"

using namespace Geometry;

double Geometry::wrap(double theta) {
	theta = std::fmod(theta, 2 * M_PI);
	if (theta > M_PI) return theta - 2 * M_PI;
	else if (theta < -M_PI) return theta + 2 * M_PI;
	else return theta;
}

double Geometry::distance(const Point &a, const Point &b) {
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double Geometry::distance_x(const Point &a, const Point &b) {
	return std::abs(a.x - b.x);
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

Point Geometry::from_cv_point(cv::Point cv_point) {
	return {cv_point.x * (1.7 / 640.0),
			1.3 - cv_point.y * (1.3 / 480.0)};
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
