#include <ros/ros.h>
#include <gnuplot-iostream.h>
#include <fstream>
#include <vector>
#include <potbot_lib/kalman_filter.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "gnuplot_example");
    ros::NodeHandle nh;

	int data_num = 1000;
	potbot_lib::KalmanFilter kf;
	Eigen::MatrixXd iniA(5,5), iniC(2,5);
	iniA.setZero(); iniC.setZero();
	kf.setA(iniA); kf.setC(iniC);
	kf.initialize();
	double dt = 0.2;

	Eigen::MatrixXd MA(3,3);
	MA<< 1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	Eigen::MatrixXd MU(2,data_num);MU.setZero();
	Eigen::MatrixXd MX(3,data_num);MX.setZero();
	Eigen::MatrixXd Xhat(5,data_num);Xhat.setZero();
	Eigen::VectorXd T(data_num);T.setZero();
	double period = 60;
	for (int i = 0; i < data_num; i++) 
	{
		double t = i*dt;
		T(i) = t;
		MU(0,i) = 0.1*sin(2*M_PI/period*t);
		MU(1,i) = 0.01*t;
	}
    for (int i = 1; i < data_num; i++) 
	{
		Eigen::VectorXd Mu = MU.col(i);
		Eigen::VectorXd Mx_pre = MX.col(i-1);
		Eigen::MatrixXd MB(3,2);
		double v = Mu(0);
		double theta = Mx_pre(2);
		MB<< dt*cos(theta), 0,
			 dt*sin(theta), 0,
			 0, dt;
		Eigen::VectorXd Mx = MA*Mx_pre+MB*Mu;
		MX.col(i) = Mx;
		
		Eigen::MatrixXd A(5,5);
		A<< 1, 0, -v*sin(theta)*dt, cos(theta)*dt, 0,
			0, 1, v*cos(theta)*dt, sin(theta)*dt, 0,
			0,0,1,0,dt,
			0,0,0,1,0,
			0,0,0,0,1;

		Eigen::MatrixXd C(2,5);
		C<< 1,0,0,0,0,
			0,1,0,0,0;

		Eigen::MatrixXd observed_data(2,1);
		observed_data<< Mx(0), Mx(1);
		kf.setA(A);
		kf.setC(C);
		std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> ans = kf.update(observed_data,dt);
		Eigen::VectorXd xhat = std::get<0>(ans);
		Eigen::MatrixXd P = std::get<1>(ans);
		Eigen::MatrixXd K = std::get<2>(ans);
		Xhat.col(i) = xhat;
    }

	std::ofstream file("data.csv");

    for (int i = 0; i < data_num; i++) {
        file << T(i) << "," << 
		MX(0,i) << "," << MX(1,i) << "," << MX(2,i) << "," << MU(0,i) << "," << MU(1,i) << "," <<
		Xhat(0,i) << "," << Xhat(1,i) << "," << Xhat(2,i) << "," << Xhat(3,i) << "," << Xhat(4,i) << "\n";
    }

    file.close();

    // Gnuplotで描画
	std::vector<std::pair<double, double>> data1;
	for (int i = 0; i < data_num; i++) 
	{
		data1.push_back(std::make_pair(MX.col(i)(0), MX.col(i)(1)));
	}
	std::vector<std::pair<double, double>> data2;
	for (int i = 0; i < data_num; i++) 
	{
		data2.push_back(std::make_pair(Xhat.col(i)(0), Xhat.col(i)(1)));
	}

	std::vector<std::pair<double, double>> data3;
	for (int i = 0; i < data_num; i++) 
	{
		data3.push_back(std::make_pair(T(i), Xhat.col(i)(2)));
	}

	Gnuplot gp;
	gp << "set title 'Two Lines'\n";            // タイトル
    gp << "set xlabel 'X-axis'\n";             // X軸ラベル
    gp << "set ylabel 'Y-axis'\n";             // Y軸ラベル
    gp << "plot '-' with lines title 'Line 1', '-' with lines title 'Line 2'\n";
    gp.send1d(data1);
    gp.send1d(data2);

	// gp << "set title 'Graph 2'\n";
    // gp << "plot '-' with lines title 'Line 2'\n";
    // gp.send1d(data3);
    

    // ROSスピン（他のノードと連携する場合）
    ros::spin();

    return 0;
}