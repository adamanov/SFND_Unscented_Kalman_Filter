#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    // if this is false,  measurements will be ignored (except during init)
    use_laser_ = true;
    use_radar_ = true;

    x_ = VectorXd(5); // initial state vector

    P_ = MatrixXd(5, 5); // initial covariance matrix
    P_.setIdentity();
    P_(0, 0) = 1;
    P_(1, 1) = 1;
    P_(2, 2) = 0.5;
    P_(3, 3) = 0.5;
    P_(4, 4) = 0.5;

    std_a_ = 5.0;     // Process noise standard deviation longitudinal acceleration in m/s^2
    std_yawdd_ = 0.8; // Process noise standard deviation yaw acceleration in rad/s^2

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

    std_laspx_ = 0.15; // Laser measurement noise standard deviation position1 in m
    std_laspy_ = 0.15; // Laser measurement noise standard deviation position2 in m

    std_radr_ = 0.3;    // Radar measurement noise standard deviation radius in m
    std_radphi_ = 0.03; // Radar measurement noise standard deviation angle in rad
    std_radrd_ = 0.3;   // Radar measurement noise standard deviation radius change in m/s

    is_initialized_ = false;

    n_x = 5;
    n_aug = 7;

    /*** ___________Generate Sigma Points____________ ***/
    Xsig = MatrixXd(n_x, 2 * n_x + 1);
    /*** _______Augmented Sigma Points_______________ ***/
    Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
    /*** _______Predict Sigma Points_______ ***/
    Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

    /***_______ Predict Mean and Covariance _________ ***/
    weights = VectorXd(2 * n_aug + 1);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    if (!is_initialized_)
    {
        std::cout << " ************** Initialize a initial state  ************** " << std::endl;
        Initialization(meas_package);
    }

    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    std::cout << " ************** Prediction Step  ************** " << std::endl;
    Prediction(delta_t);

    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        std::cout << " ************** Update Step Lidar ************** " << std::endl;
        UpdateLidar(meas_package);
    }
    else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        std::cout << " ************** Update Step Radar ************** " << std::endl;
        UpdateRadar(meas_package);
    }
    else
        std::cout << "Please select a measurements type, either Lidar or Radar" << std::endl;
}

void UKF::Prediction(double delta_t)
{
    /*** ___________Generate Sigma Points____________ ***/
    std::cout << "Generate Sigma Points" << std::endl;
    GenerateSigmaPoints(&Xsig);
    /***______________________________________________***/

    /*** _______Augmented Sigma Points_______________ ***/
    std::cout << "Augmented Sigma Points" << std::endl;
    AugmentedSigmaPoints(&Xsig_aug);
    /***______________________________________________***/

    /*** _______Predict Sigma Points_______ ***/
    std::cout << "Predict Sigma Points" << std::endl;
    SigmaPointPrediction(&Xsig_pred);
    /***______________________________________________***/

    /***_______ Predict Mean and Covariance _________ ***/
    std::cout << "Predict Mean and Covariance " << std::endl;
    PredictMeanAndCovariance(&x_, &P_);
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /// Define spreading parameter
    lambda = 3 - n_aug;
    n_z = 2;
    Eigen::VectorXd z = Eigen::VectorXd(n_z);
    // switch (meas_package.sensor_type_)
    // {
    // case MeasurementPackage::LASER:
    //     if (use_laser_)
    //     {
    //         std::cout << " ---- Lidar Measurements Vector ---" << std::endl;
    //         z = Eigen::VectorXd(n_z);
    //         z = meas_package.raw_measurements_;
    //         // meas_package.raw_measurements_[0],   // dx in m
    //         // meas_package.raw_measurements_[1],   // dy in m
    //     }
    // }

    z = meas_package.raw_measurements_;
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1).setZero(); /// (3*15)

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z).setZero();

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z).setZero();

    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        /// State Vector for simplicity
        double p_x = Xsig_pred.col(i)[0];
        double p_y = Xsig_pred.col(i)[1];
        double v = Xsig_pred.col(i)[2];
        double phi = Xsig_pred.col(i)[3];
        double phi_d = Xsig_pred.col(i)[4];

        /// Put back into Zsig Matrix
        Zsig(0, i) = p_x;
        Zsig(1, i) = p_y;
    }

    /// Predicted Measurement Mean
    for (int i = 0; i < 2 * n_aug + 1; i++)
        z_pred = z_pred + weights(i) * Zsig.col(i);

    /// Predicted Covariance
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        VectorXd zDiff = Zsig.col(i) - z_pred;

        while (zDiff(1) > M_PI)
            zDiff(1) -= 2. * M_PI;
        while (zDiff(1) < -M_PI)
            zDiff(1) += 2. * M_PI;
        S = S + weights(i) * zDiff * zDiff.transpose();
    }

    MatrixXd R = MatrixXd(n_z, n_z);
    R << pow(std_laspx_, 2), 0,
        0, pow(std_laspy_, 2);

    S = S + R;

    VectorXd weights = VectorXd(2 * n_aug + 1); /// (15x1)
    weights(0) = lambda / (lambda + n_aug);

    for (int i = 1; i < 2 * n_aug + 1; i++)
        weights(i) = 1 / (2 * (lambda + n_aug));

    /// create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z).setZero();

    /// calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        VectorXd xDiff = Xsig_pred.col(i) - x_;
        while (xDiff(3) > M_PI)
            xDiff(3) -= 2. * M_PI;
        while (xDiff(3) < -M_PI)
            xDiff(3) += 2. * M_PI;

        VectorXd zDiff = Zsig.col(i) - z_pred;
        while (zDiff(1) > M_PI)
            zDiff(1) -= 2. * M_PI;
        while (zDiff(1) < -M_PI)
            zDiff(1) += 2. * M_PI;

        Tc = Tc + weights(i) * xDiff * zDiff.transpose();
    }

    /// calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x, n_z).setZero();
    K = Tc * S.inverse();

    // residual
    std::cout << "\nz= \n " << z << " \n"
              << "z_pred= \n"
              << z_pred << std::endl;

    VectorXd z_diff = z - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI)
        z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
        z_diff(1) += 2. * M_PI;

    /// update state mean and covariance matrix
    x_ = x_ + K * z_diff;

    P_ = P_ - K * S * K.transpose();

    // print result
    std::cout << "\nUpdated state x =  " << std::endl
              << x_ << std::endl;
    std::cout << "\nUpdated state covariance P = " << std::endl
              << P_ << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{

    /// Define spreading parameter
    lambda = 3 - n_aug;

    n_z = 3;
    Eigen::VectorXd z =Eigen::VectorXd(n_z);
//switch (meas_package.sensor_type_)
//{
//case MeasurementPackage::RADAR:
//    std::cout << " ---- Radar Measurements Vector ---" << std::endl;
//    z = Eigen::VectorXd(n_z).setZero();
//    z = meas_package.raw_measurements_;
//    // meas_package.raw_measurements_[0],   // rho in m
//    // meas_package.raw_measurements_[1],   // phi in rad
//    // meas_package.raw_measurements_[2];   // rho_dot in m/s
//}
 z = meas_package.raw_measurements_;

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z).setZero();

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z).setZero();
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1).setZero(); /// (3*15)

    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        /// State Vector for simplicity
        double p_x = Xsig_pred.col(i)[0];
        double p_y = Xsig_pred.col(i)[1];
        double v = Xsig_pred.col(i)[2];
        double phi = Xsig_pred.col(i)[3];
        double phi_d = Xsig_pred.col(i)[4];

        /// Radar Measurement Vector
        double rho = sqrt(pow(p_x, 2) + pow(p_y, 2));
        double phi_rdr = atan2(p_y, p_x);
        double rho_d = (p_x * cos(phi) * v + p_y * sin(phi) * v) / rho;

        //std::cout<<rho <<" " << phi_rdr <<" " << rho_d<<std::endl;

        /// Put back into Zsig Matrix
        Zsig(0, i) = rho;
        Zsig(1, i) = phi_rdr;
        Zsig(2, i) = rho_d;
    }

    /// Predicted Measurement Mean
    for (int i = 0; i < 2 * n_aug + 1; i++)
        z_pred = z_pred + weights(i) * Zsig.col(i);

    /// Predicted Covariance
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        VectorXd zDiff = Zsig.col(i) - z_pred;

        while (zDiff(1) > M_PI)
            zDiff(1) -= 2. * M_PI;
        while (zDiff(1) < -M_PI)
            zDiff(1) += 2. * M_PI;
        S = S + weights(i) * zDiff * zDiff.transpose();
    }

    MatrixXd R = MatrixXd(n_z, n_z);
    R << pow(std_radr_, 2), 0, 0,
        0, pow(std_radphi_, 2), 0,
        0, 0, pow(std_radrd_, 2);

    S = S + R;


    /// create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z).setZero();

    /// calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug + 1; i++)
    {
        VectorXd xDiff = Xsig_pred.col(i) - x_;
        while (xDiff(3) > M_PI)
            xDiff(3) -= 2. * M_PI;
        while (xDiff(3) < -M_PI)
            xDiff(3) += 2. * M_PI;

        VectorXd zDiff = Zsig.col(i) - z_pred;
        while (zDiff(1) > M_PI)
            zDiff(1) -= 2. * M_PI;
        while (zDiff(1) < -M_PI)
            zDiff(1) += 2. * M_PI;

        Tc = Tc + weights(i) * xDiff * zDiff.transpose();
    }

    /// calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x, n_z).setZero();
    K = Tc * S.inverse();

    // residual
    std::cout << "\nz= \n " << z << " \n"
              << "z_pred= \n"
              << z_pred << std::endl;

    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI)
        z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI)
        z_diff(1) += 2. * M_PI;

    /// update state mean and covariance matrix
    x_ = x_ + K * z_diff;

    P_ = P_ - K * S * K.transpose();

    // print result
    std::cout << "\nUpdated state x: " << std::endl
              << x_ << std::endl;
    std::cout << "\nUpdated state covariance P: " << std::endl
              << P_ << std::endl;
}

void UKF::Initialization(MeasurementPackage meas_package)
{
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {
        is_initialized_ = true;
        std::cout << "Using a Lidar: " << is_initialized_ << std::endl;
        std::cout << meas_package.raw_measurements_ << std::endl;

        x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1],
            0,
            0,
            0;
        time_us_ = meas_package.timestamp_;
        std::cout << "Initial state vector: \n"
                  << x_ << std::endl;
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
        is_initialized_ = true;
        std::cout << "Using a Radar:  " << is_initialized_ << std::endl;
        std::cout << meas_package.raw_measurements_ << std::endl;

        /// Convert radar from polar to cartesian coordinates and initialize state
        double rho = meas_package.raw_measurements_[0];
        double phi_rd = meas_package.raw_measurements_[1];
        double rho_d = meas_package.raw_measurements_[2];

        std::cout << rho << phi_rd << rho_d << std::endl;

        double px = cos(phi_rd) * rho;
        double py = sin(phi_rd) * rho;

        x_ << px,
            py,
            0,
            0,
            0;

        std::cout << "Initial state vector: \n"
                  << x_ << std::endl;
        time_us_ = meas_package.timestamp_;
    }

    else
        std::cout << "Could not detect a measurement sensor type" << std::endl;
}

void UKF::GenerateSigmaPoints(Eigen::MatrixXd *Xsig_out)
{
    /// calculate square root of P
    MatrixXd A = P_.llt().matrixL(); // square matrix
    MatrixXd P_root = P_.llt().matrixLLT();

    /// calculate sigma points ...
    /// set sigma points as columns of matrix Xsig

    std::cout << "\nUpdated state x: " << std::endl
              << x_ << std::endl;
    std::cout << "\nUpdated state covariance P: " << std::endl
              << P_ << std::endl;

    Xsig.col(0) = x_;
    // define spreading parameter
    lambda = 3 - n_x;

    for (int i = 0; i < n_x; i++)
    {
        Xsig.col(i + 1) = x_ + sqrt((lambda + n_x)) * P_root.col(i);
        Xsig.col(i + 1 + n_x) = x_ - sqrt((lambda + n_x)) * P_root.col(i);
    }

    std::cout << "Xsig = " << std::endl
              << Xsig << std::endl;
    *Xsig_out = Xsig;
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd *Xsig_out)
{
    // set state dimension (n_x_)
    // set augmented dimension (n_aug_)
    // Process noise standard deviation longitudinal acceleration (std_a_) in m/s^2
    // Process noise standard deviation yaw acceleration (std_yawdd_) in rad/s^2

    /// define spreading parameter
    lambda = 3 - n_aug;

    /// create augmented mean vector

    VectorXd x_aug_ = VectorXd(n_aug);

    /// create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug, n_aug); /// (7x7)

    /// create sigma point matrix

    /// create augmented mean state
    x_aug_(6) = 0;
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;

    /// create augmented covariance matrix
    P_aug.setZero(); /// P_aug.fill(0.0);

    P_aug.topLeftCorner(5, 5) = P_;
    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    /// create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    /// create augmented sigma points
    Xsig_aug.col(0) = x_aug_;
    for (int i = 0; i < n_aug; ++i)
    {
        Xsig_aug.col(i + 1) = x_aug_ + sqrt(lambda + n_aug) * L.col(i);
        Xsig_aug.col(i + 1 + n_aug) = x_aug_ - sqrt(lambda + n_aug) * L.col(i);
    }

    std::cout << "Xsig_aug = " << std::endl
              << Xsig_aug << std::endl;
    *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(Eigen::MatrixXd *Xsig_out)
{
    std::cout << Xsig_out->cols() << " " << Xsig_out->rows() << std::endl;

    for (int i = 0; i < 2 * n_aug + 1; ++i)
    {
        // extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        // predicted state values
        double px_p, py_p;

        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else
        {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }
        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        // write predicted sigma point into right column
        Xsig_pred(0, i) = px_p;
        Xsig_pred(1, i) = py_p;
        Xsig_pred(2, i) = v_p;
        Xsig_pred(3, i) = yaw_p;
        Xsig_pred(4, i) = yawd_p;
    }

    std::cout << "Xsig_pred = " << std::endl
              << Xsig_pred << std::endl;

    *Xsig_out = Xsig_pred;
}

void UKF::PredictMeanAndCovariance(Eigen::VectorXd *x_pred, Eigen::MatrixXd *P_pred)
{
    lambda = 3 - n_aug;

    weights(0) = lambda / (lambda + n_aug);
    for (int i = 1; i < 2 * n_aug + 1; ++i)
        weights(i) = 1 / (2 * (lambda + n_aug));

    // Predict State Mean
    VectorXd x = VectorXd(n_x).setZero();
    for (int i = 0; i < 2 * n_aug + 1; ++i)
        x = x + weights(i)*Xsig_pred.col(i);

    // Predicted State Covariance Matrix;
    MatrixXd P = MatrixXd(n_x,n_x).setZero();
    for (int i =0 ; i<2 * n_aug + 1; ++i)
    {
        // State Difference
        VectorXd xDiff = Xsig_pred.col(i) - x;

        // angle normalization
        /* the state contains an angle. As you have learned before, subtracting angles is a problem for Kalman filters,
         * because the result might be 2π plus a small angle, instead of just a small angle.
         */

        while (xDiff(3)> M_PI) xDiff(3)-=2.*M_PI;
        while (xDiff(3)<-M_PI) xDiff(3)+=2.*M_PI;

        // Predicted Covariance
        P = P + weights(i) * xDiff * xDiff.transpose() ;
    }
    std::cout << "x_ = " << std::endl << x_ << std::endl;
    std::cout << "P_ = " << std::endl << P_ << std::endl;

    *x_pred = x_;
    *P_pred = P_;
}
