#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

typedef struct measurementPackage {
	enum SensorType{
		LASER,
		RADAR
	} sensor_type;
	long long timestamp;
	Eigen::VectorXd raw_measurements;
} t_measurementPackage;

#endif // MEASUREMENT_PACKAGE_H_
