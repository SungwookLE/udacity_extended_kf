# Writeup: How to implement this Extended Kalman Filter Code
Self-Driving Car Engineer Nanodegree Program

**Thesis: Kalman Filter with Radar, Lidar using C++, Eigen LIB**
**Writer: SungwookLE**
**Date: '20.12/30**

## Code Navigation

### 1. main.cpp
- Line 36~39:
The thing you may attention is fusionEKF is the class for implementing fusion flow structure with Radar and Lidar(Laser) data.
tools is class and tools class include calculating jacobian matrix for linearization(extended algorithm) and RMSE error.
When declaring the fusionEKF class, the constructor is activated.(look at the FusionEKF.cpp)

- Line 43: 
h is instance of class """uWS::Hub""", and this is for communicate with simulator (data in & out).
h.onMessage is member function of Hub class. And the arguments of that member are fusionEKF, tools, ... and so on.

- Line 70~79:
This code is for packaging the Lidar data(Laser). And Lidar data consists of position x and position y.

- Line 80~91:
Packaging the Radar data. And Radar data consists of distance R and angle theta, velocity of distance dot_R

- Line111:
fusionEKF calls the ProcessMeasurement(meas_package) and ProcessMeasurement is the memeber fuction for kalman fusing processing
This is the key point of this project. Let's look at the FusionEKF.cpp

### 2. FusionEKF.cpp with kalman_filter.cpp
- Line84:
There is **FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)** that is called in main.
Above this, There is constructor of this class.(Line15)
**FusionEKF::FusionEKF()**
when initialized this class, this constructor is activated.

R is measurement covariance matrix. In this problem, R_laser is 2 by 2 matrix (each is px, py measurement covariance)
R_radar is 3 by 3 matrix (each is rho, theta, rho_dot)
The bigger R is, The more non-reliable the measurement value.

H is output equation matrix. In this problem, H_laser is 2 by 4 matrix. and H_radar is non-linear function (triangular gemetry)
So. actuallay H_radar can be expressed by function of H_radar. and it can not expressed by linear matrix. 
To solve this non-linearity, linearization using Jacobian is done in tools class. and the jacobian results of function of H_radar is Hj_ matrix.
The size of Hj_ is 3 by 4.

**ekf_.x_ << 0,0,0,0** means all initial variable set to be zero.
ekf_.P_ is the covariance of the kalman filter, the bigger element is, the faster converge the error to zero.

ekf_.F_ is the System Matrix.
This matrix express the relationship of posion and velocity to each direction x and y.

From Now on,
**void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack**
- Line 88~122:
This is the first loop when kalman filter is acting.
- Line 102~109:
If Radar data is acquised, The data set consists of rho, theta, rho_dot.
Therefore, to convert to catesian coordinate, cosine and sine calculation is done.

- Line 110~115:
If Lidar data is acquired, The ekf_.x_ is set to lidar data in first loop process.

- Line 118: 
previous_timestamp_ is updated current timestamps.
Note: This is the point. In first trail, My code diverge. and I found this line was undone. After I added it, the code performance converge

- Line 138~139:
ekf_.F_(0,2), (1,3) set to dt. dt is time interval.
And ekf_.Q_ is set according to noise ax, noise ay variance

- Line 153:
ekf_.Predict() is done. and Predict() member is written in "kalman_filter.cpp".
In predict member, x_ is calcualted by F matrix product X vector **(i.e. x_ = F_ *X_+ u, where u is zero)**
Additionally, Covariance P is updated by **P_ = F_ * P_ * F_.transpose() + Q_;**

- Line 165 ~ 172:
if Radar data is coming, as i said above, the H_radar is non-linear function and needed to be linearize.
Therefore tools.CalculateJacobian(ekf_.X_) is calculating jacobian at point x.
And set the ekf_ matirx using ekf_.Init(), and call the **ekf_.UpdateEKF(measurement_pack.raw_measurements_)**
Let's look at the UpdateEKF memeber in "kalman_filter.cpp".
in line 81~115, px, py, vx, vy are converted to rho, phi, rho_dot.
and phi set in range from **'-pi to +pi'**

In line 117~121, kalman processing is implemented for radar processing.

>>>	Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_; // 3x3
>>>	Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse(); // 4x4 * 4x3 * x 3x3 = 4x3
>>>	x_ = x_ + K * y;
>>>	P_ = (I - K * H_) * P_;


x is updated according to error y and kalman gain K.
P is updated.

- Line 173~ 179:
if Lidar data is coming, the linear kalman update is done using ekf_.Update().

>>>	Eigen::VectorXd y = z - H_ * x_; // error
>>>	Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
>>>	Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
>>>	x_ = x_ + K * y;
>>>	P_ = (I - K * H_) * P_;


- Line 181~183:
print the state 'x' vector, and covariance P matrix.
P must be converge while loop.

### Again, 3. main.cpp
- Line 130: tools.CalculateRMSE() is called.
calculateRMSE is implemented "tools.cpp".
RMSE is mean squared error sum.


>>>    for (int i; i < estimations.size(); ++i)
>>>    {
>>>       error_square = (estimations.at(i) - ground_truth.at(i)).array()* (estimations.at(i) - ground_truth.at(i)).array(); 
>>>      RMSE += error_square;
>>>    }

>>>   RMSE = RMSE / estimations.size();
>>>   RMSE = error_square.array().sqrt();

See that array() member. it is implemented Eigne LIB. and it makes each element product calculation.

- LIne 131~151:
The message send to simulator using message frame.

All done.
Radar and Lidar SensorFusion using KalmanFilter is implemented.

### The Final Performance 
Under Simulator, px, py, vx, and vy RMSE are less than the values **[.11, .11, 0.52, 0.52].**
PASS!!!
