#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>  // Eigenライブラリを使用

using namespace std;
using namespace Eigen;

// 目標曲線の点列
vector<Vector2d> targetCurve = {Vector2d(0.0, 0.0), Vector2d(1.0, 1.0), Vector2d(2.0, 0.5)};

// 比較曲線の初期点列（ランダムな場合）
vector<Vector2d> initialComparisonCurve = {Vector2d(0, 0), Vector2d(0, 0), Vector2d(0, 0)};

// 学習率
double learningRate = 0.1;

// L2ノルムを計算する関数
double calculateL2Norm(const vector<Vector2d>& curve1, const vector<Vector2d>& curve2) {
    double sum = 0.0;
    for (size_t i = 0; i < curve1.size(); ++i) {
        Vector2d diff = curve1[i] - curve2[i];
        sum += diff.norm();  // L2ノルムを加算
    }
    return sum;
}

// 勾配効果法による探索
void gradientDescent(vector<Vector2d>& comparisonCurve) {
    size_t maxIterations = 1000;
    double tolerance = 1e-6;  // 収束基準

    for (size_t iter = 0; iter < maxIterations; ++iter) {
        // 目標曲線との差の勾配を計算
        vector<Vector2d> gradient(comparisonCurve.size(), Vector2d::Zero());
        for (size_t i = 0; i < comparisonCurve.size(); ++i) {
            Vector2d diff = comparisonCurve[i] - targetCurve[i];
            gradient[i] = 2 * diff;  // 勾配は差の2倍
            // gradient[i] = diff / (diff.norm()+1e-100);  // 厳密な勾配
        }

        // 曲線を更新
        for (size_t i = 0; i < comparisonCurve.size(); ++i) {
            comparisonCurve[i] -= learningRate * gradient[i];
        }

        // 収束判定
        double error = calculateL2Norm(comparisonCurve, targetCurve);
        if (error < tolerance) {
            cout << "Converged at iteration " << iter << ", error = " << error << endl;
            break;
        }
    }
}

int main(int argc, char** argv) {
	// ros::init(argc, argv, "grad");
    vector<Vector2d> comparisonCurve = initialComparisonCurve;

    // 勾配効果法による比較曲線の探索
    gradientDescent(comparisonCurve);

    // 最終的な比較曲線の表示
    cout << "Final comparison curve:" << endl;
    for (const auto& point : comparisonCurve) {
        cout << "(" << point.x() << ", " << point.y() << ")" << endl;
    }

    return 0;
}