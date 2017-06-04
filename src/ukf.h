#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "sigma_point_manager.h"
#include "entity.h"
#include "transformer.h"
#include "ctrv_to_laser_transformer.h"
#include "ctrv_to_radar_transformer.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF
{
public:

	SigmaPointManager sigma_manager_;

	bool is_initialized_;
	bool use_laser_;
	bool use_radar_;

	double previous_time_stamp_;
	double delta_time_;

	Entity state_entity_;
	Entity radar_entity_;
	Entity laser_entity_;

	CTRVToLaserTransformer laser_transformer_;
	CTRVToRadarTransformer radar_transformer_;

	VectorXd x_;
	MatrixXd P_;

	double std_a_;
	double std_yawdd_;

	VectorXd weights_;
	double lambda_;

	int n_x_;
	int n_aug_;
	int n_sigma_;

	long long time_us_;
	double NIS_radar_;
	double NIS_laser_;

	UKF();
	virtual ~UKF();
	void ProcessMeasurement(MeasurementPackage meas_package);
	void PrintNIS();

	private:
	void UpdateDeltaTime(double time_stamp);
	void PredictSensorCovariance(Entity* sensor);
	void PredictSensorMean(Entity* sensor);
	void PredictState(double delta_t);
	void ProcessWithSensor(MeasurementPackage meas_package, Entity& entity, Transformer& transformer);
	void ConvertPredictionToSensorSpace(Entity& state, Entity* sensor, Transformer& transformer);
	void UpdateStateWithSensor(MeasurementPackage& meas_package, Entity& state, Entity& sensor);
	MatrixXd CalculateCrossCorrelation(Entity& state, Entity& sensor);
	static void CalculateNIS(MeasurementPackage& meas_package, Entity& sensor);

};

#endif
