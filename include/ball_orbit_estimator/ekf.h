#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
// #include <iostream>
#include <utility>

namespace Filter
{
/*
 * @brief 非線型カルマンフィルター(線型カルマンフィルターとしても機能させられる)
 * xを現在の状態, uを入力, x_preを前時刻の状態, Gをノイズの線型変換行列, wをN(0,
 * Q)に従うノイズとする
 * 状態方程式: x = F x_pre + u + G w
 * zを観測値, Hを観測行列, vをN(0, R)に従う観測ノイズとする
 * 観測方程式: z = H x + v
 * @author future731
 */
class EKF
{
public:
  using VecXd = Eigen::VectorXd;
  using MatXd = Eigen::MatrixXd;
  // @brief初期状態と分散共分散行列の初期値
  explicit EKF(const VecXd& x_init, const MatXd& P_init) : m_x_filtered(x_init), m_P_filtered(P_init)
  {
  }

  /*
   * @brief 現在状態を推定し、現在の状態の分散共分散行列を求める
   * @param f 状態方程式の状態更新関数
   * @param F 状態方程式の状態更新関数を状態で偏微分したヤコビ行列
   * @param G 状態方程式のノイズ線型変換行列
   * @param Q 状態方程式のノイズが従う分散共分散行列
   * @param u 状態方程式の入力
   * @param z 観測値
   * @param h 観測方程式の観測値更新関数
   * @param dh
   * 前状態と入力を受け取って観測方程式の観測値更新関数を状態で偏微分したヤコビ行列を返す関数
   * @param R 観測値のノイズが従う分散共分散行列
   * @note 線型カルマンフィルターとして機能させるには
   * 関数fを[&](const VecXd& x) { return F * x; }, 関数hを[&](const VecXd& x) {
   * return H * x; }とするとよい
   */
  std::pair<VecXd, MatXd> update(std::function<VecXd(VecXd)> f, const MatXd& F, const MatXd& G, const MatXd& Q,
                                 const VecXd& u, const VecXd& z, std::function<VecXd(VecXd)> h,
                                 std::function<MatXd(VecXd)> dh, const MatXd& R)
  {
    // 予測段
    VecXd x_pred = f(m_x_filtered) + u;
    MatXd P_pred = F * m_P_filtered * F.transpose() + G * Q * G.transpose();
    // 更新段
    VecXd e = z - h(x_pred);                         // 観測残差
    MatXd H = dh(m_x_filtered);                      // ヤコビ行列
    MatXd S = R + H * P_pred * H.transpose();        // 観測残差の分散共分散行列
    MatXd K = P_pred * H.transpose() * S.inverse();  // 最適カルマンゲイン
    m_x_filtered = x_pred + K * e;                   // 現在の状態予測
    Eigen::Index size = (K * H).cols();
    m_P_filtered = (MatXd::Identity(size, size) - K * H) * P_pred;  // 現在の分散共分散行列
    /*
    std::cout << "ekf update: " << std::endl;
    std::cout << "F" << std::endl;
    std::cout << F << std::endl;
    std::cout << "G" << std::endl;
    std::cout << G << std::endl;
    std::cout << "Q" << std::endl;
    std::cout << Q << std::endl;
    std::cout << "u" << std::endl;
    std::cout << u << std::endl;
    std::cout << "z" << std::endl;
    std::cout << z << std::endl;
    std::cout << "R" << std::endl;
    std::cout << R << std::endl;
    std::cout << "f(m_x_filtered)" << std::endl;
    std::cout << f(m_x_filtered) << std::endl;
    std::cout << "x_pred" << std::endl;
    std::cout << x_pred << std::endl;
    std::cout << "P_pred" << std::endl;
    std::cout << P_pred << std::endl;
    std::cout << "e" << std::endl;
    std::cout << e << std::endl;
    std::cout << "H" << std::endl;
    std::cout << H << std::endl;
    std::cout << "S" << std::endl;
    std::cout << S << std::endl;
    std::cout << "K" << std::endl;
    std::cout << K << std::endl;
    std::cout << "m_x_filtered" << std::endl;
    std::cout << m_x_filtered << std::endl;
    std::cout << "size" << std::endl;
    std::cout << size << std::endl;
    std::cout << "m_P_filtered" << std::endl;
    std::cout << m_P_filtered << std::endl;
    */
    std::pair<VecXd, MatXd> result;
    result.first = m_x_filtered;
    result.second = m_P_filtered;
    return result;
  }

private:
  VecXd m_x_filtered;
  MatXd m_P_filtered;
};

}  // namespace Filter
