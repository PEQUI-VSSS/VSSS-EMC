#ifndef VSSS_GEOMETRY_H
#define VSSS_GEOMETRY_H

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace Geometry {
	struct Point;

	struct Vector {
		double size;
		double theta;

		Vector(const double size, const double theta) : size(size), theta(theta) {}
		explicit Vector(const Point &p);
		Vector operator*(double value);
		Vector unitary();
		Vector with_size(double new_size);
	};

	struct Point {
		double x;
		double y;

		friend std::ostream& operator<<(std::ostream& os, const Point& p) {
			os << "{ " << p.x << ", " << p.y << " }";
			return os;
		}

		constexpr Point(double x, double y) noexcept : x(x), y(y) {};
		constexpr Point() noexcept : x(0), y(0) {};
		Vector operator-(const Point &p2) const;
		Point operator+(const Vector &v) const;
		Point operator-(const Vector &v) const;
		cv::Point to_cv_point();
	};

	struct Line {
		double a = 0;
		double b = 0;

		constexpr Line(Point p1, Point p2) noexcept {
			if(p2.x == p1.x) {
				a = (p2.y - p1.y)/(p2.x - p1.x + 0.00001);
			} else {
				a = (p2.y - p1.y)/(p2.x - p1.x);
			}
			b = p1.y - a * p1.x;
		}
	};

	struct Segment {
		Point a;
		Point b;
	};

	constexpr Point middle(const Point& p1, const Point &p2) noexcept {
		return {(p1.x + p2.x)/2, (p1.y + p2.y)/2};
	}

	constexpr double degree_to_rad(double degree) {
		return degree * M_PI / 180.0;
	}

//	Define operator* de forma associativa (value * v == v * value)
//	Vector operator*(double value, Vector v) {return v * value;}

	double distance(const Point &a, const Point &b);
	double distance_x(const Point &a, const Point &b);
	double distance_y(const Point &a, const Point &b);
	Point intersection(const Line &l1, const Line &l2);
	Point from_cv_point(cv::Point cv_point);
	Point from_cv_point(double x, double y);
	double wrap(double theta);

	struct Rectangle {
		std::array<Point, 4> points;

		static Rectangle from_robot(Point p, Vector to_front, double offset);
		bool intersect(const Rectangle& b);
	};

}

#endif //VSSS_GEOMETRY_H
