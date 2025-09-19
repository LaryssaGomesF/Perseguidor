#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    // Inicializa os parâmetros. Estes são valores iniciais comuns.
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    // Inicializa o estado
    angle = 0.0f;
    bias = 0.0f;

    // Inicializa a matriz de covariância do erro
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
}

float KalmanFilter::update(float newAngle, float newRate, float dt) {
    // -------- Etapa de PREDIÇÃO --------
    // Estima o novo estado com base no giroscópio
    
    // 1. Prediz o ângulo
    angle += dt * (newRate - bias);

    // 2. Prediz a matriz de covariância do erro
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // -------- Etapa de CORREÇÃO (ATUALIZAÇÃO) --------
    // Corrige a predição usando a medição do acelerômetro

    // 1. Calcula o erro (inovação)
    float y = newAngle - angle;

    // 2. Calcula a covariância do erro da inovação
    float S = P[0][0] + R_measure;

    // 3. Calcula o Ganho de Kalman
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 4. Atualiza a estimativa do ângulo com a medição
    angle += K[0] * y;

    // 5. Atualiza a estimativa do bias do giroscópio
    bias += K[1] * y;

    // 6. Atualiza a matriz de covariância do erro
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}
