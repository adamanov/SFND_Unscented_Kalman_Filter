

#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
public:
    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);
    void Prediction(double delta_t);
    void UpdateLidar(MeasurementPackage meas_package);
    void UpdateRadar(MeasurementPackage meas_package);

    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // Augumented state vector
    Eigen::VectorXd x_aug;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // Generate A Sigma Point
    Eigen::MatrixXd Xsig;

    // Augment Sigma Points
    Eigen::MatrixXd Xsig_aug;

    // predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred;


    // time when the state is true, in us
    long long time_us_;

    double std_a_;    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_yawdd_;     // Process noise standard deviation yaw acceleration in rad/s^2

    double std_laspx_;     // Laser measurement noise standard deviation position1 in m
    double std_laspy_;     // Laser measurement noise standard deviation position2 in m

    double std_radr_;      // Radar measurement noise standard deviation radius in m
    double std_radphi_;      // Radar measurement noise standard deviation angle in rad
    double std_radrd_ ;     // Radar measurement noise standard deviation radius change in m/s


    // Weights of sigma points
    Eigen::VectorXd weights;

    // State dimension
    int n_x;

    // Augmented state dimension
    int n_aug;

    // Measurement dimenension
    int n_z;

    // Sigma point spreading parameter
    double lambda;

    //time diff in sec
    double delta_t;

    // NIS values
    // the current NIS for radar
    double NIS_radar_;

    // the current NIS for laser
    double NIS_laser_ ;

    void Initialization(MeasurementPackage meas_package);

    void GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out);
    void AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);
    void SigmaPointPrediction(Eigen::MatrixXd* Xsig_out);
    void PredictMeanAndCovariance(Eigen::VectorXd* x_pred,
                                  Eigen::MatrixXd* P_pred);
    void PredictRadarMeasurement(Eigen::VectorXd* z_out,
                                 Eigen::MatrixXd* S_out,
                                 Eigen::MatrixXd* Zsig_out);

    void PredictLidarMeasurement(Eigen::VectorXd *x_out,
                                 Eigen::MatrixXd *P_out,
                                 MeasurementPackage meas_package);

    void UpdateState(Eigen::VectorXd* x_out,
                     Eigen::MatrixXd* P_out,
                     MeasurementPackage meas_package,
                     Eigen::VectorXd z_out,
                     Eigen::MatrixXd S_out,
                     Eigen::MatrixXd Zsig,
                     Eigen::VectorXd z);


};

#endif  // UKF_H