// KalmanFilter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
    // Construtor: inicializa as variáveis do filtro
    KalmanFilter();

    /**
     * @brief Executa um passo do filtro de Kalman.
     * @param newAngle Ângulo medido pelo acelerômetro (em graus).
     * @param newRate Velocidade angular medida pelo giroscópio (em graus/segundo).
     * @param dt O intervalo de tempo desde a última chamada (em segundos).
     * @return O ângulo filtrado (em graus).
     */
    float update(float newAngle, float newRate, float dt);

private:
    // --- Parâmetros do Filtro (Ajustáveis) ---
    // Estes valores determinam o quão "confiante" o filtro é em cada sensor.
    // Você pode precisar ajustá-los experimentalmente.
    float Q_angle;  // Incerteza do processo para o ângulo
    float Q_bias;   // Incerteza do processo para o bias do giroscópio
    float R_measure;// Incerteza da medição (ruído do acelerômetro)

    // --- Variáveis de Estado do Filtro ---
    float angle;      // O ângulo estimado
    float bias;       // O bias do giroscópio estimado
    float P[2][2];    // Matriz de covariância do erro
};

#endif // KALMAN_FILTER_H
