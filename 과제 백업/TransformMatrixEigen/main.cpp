#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#define DEG2RAD(angle) ((angle) * M_PI / 180.0)

using namespace std;
using namespace Eigen;

// 동차변환 행렬 생성 함수
Matrix4d createTransformMatrix(double x, double y, double theta_deg) {
    double theta = DEG2RAD(theta_deg); // 각도를 라디안으로 변환
    Matrix4d T = Matrix4d::Identity();
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta);
    T(0, 3) = x;
    T(1, 3) = y;
    return T;
}

int main() {
    // A, B, C의 위치와 방향
    Vector3d A(3, 4, 45);
    Vector3d B(-6, 7, -60);
    Vector3d C(10, 2, 135);

    // A->B 동차변환행렬
    Matrix4d T_A_B = createTransformMatrix(B(0), B(1), B(2));

    // C->B 동차변환행렬
    Matrix4d T_C_B = createTransformMatrix(B(0) - C(0), B(1) - C(1), B(2) - C(2));

    // A->C 동차변환행렬 계산 (T_A_C = T_C_B⁻¹ * T_A_B)
    Matrix4d T_A_C = T_C_B.inverse() * T_A_B;

    // 결과 출력
    cout << "A->B Transformation Matrix:\n" << T_A_B << endl;
    cout << "C->B Transformation Matrix:\n" << T_C_B << endl;
    cout << "A->C Transformation Matrix:\n" << T_A_C << endl;

    return 0;
}

