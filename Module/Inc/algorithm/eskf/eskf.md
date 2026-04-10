# ESKF 姿态解算数学原理（ENU-FLU，gyro predict + accel update + yaw-only mag update）

> 这份文档只针对当前这套 **姿态 ESKF**：
>
> - 导航系：**ENU**
> - 机体系：**FLU**
> - 名义状态：\((q_{nb}, b_g, b_a)\)
> - 误差状态：\((\delta\theta, \delta b_g, \delta b_a)\)
> - 过程：**gyro predict**
> - 观测：**accel update**
> - 观测：**yaw-only mag update**
>
> 文档中的符号、正负号、注入方式，统一按当前已经验证通过的实现约定书写。

---

# 1. 坐标系、旋转与符号约定

## 1.1 坐标系

### 导航系 \(n\)：ENU

$$
e_E^n = \begin{bmatrix}1\\0\\0\end{bmatrix},\quad
e_N^n = \begin{bmatrix}0\\1\\0\end{bmatrix},\quad
e_U^n = \begin{bmatrix}0\\0\\1\end{bmatrix}
$$

### 机体系 \(b\)：FLU

$$
e_F^b = \begin{bmatrix}1\\0\\0\end{bmatrix},\quad
e_L^b = \begin{bmatrix}0\\1\\0\end{bmatrix},\quad
e_U^b = \begin{bmatrix}0\\0\\1\end{bmatrix}
$$

## 1.2 旋转矩阵定义

定义

$$
v^n = C_{nb} v^b
$$

即 \(C_{nb}\) 表示 **body \(\to\) nav** 的旋转。

因此

$$
C_{bn} = C_{nb}^\top
$$

## 1.3 叉乘矩阵

对任意 \(a = [a_x, a_y, a_z]^\top\)，定义

$$
[a]_\times =
\begin{bmatrix}
0 & -a_z & a_y\\
a_z & 0 & -a_x\\
-a_y & a_x & 0
\end{bmatrix}
$$

满足

$$
[a]_\times b = a \times b
$$

## 1.4 四元数约定

四元数写作

$$
q =
\begin{bmatrix}
q_w\\q_x\\q_y\\q_z
\end{bmatrix}
$$

表示 \(C_{nb}\)。

## 1.5 小旋转指数映射

若小旋转向量为 \(\delta\theta \in \mathbb{R}^3\)，定义旋转矩阵指数映射

$$
\mathrm{Exp}(\delta\theta)
= I
+ \frac{\sin\theta}{\theta}[\delta\theta]_\times
+ \frac{1 - \cos\theta}{\theta^2}[\delta\theta]_\times^2
$$

其中

$$
\theta = \lVert \delta\theta \rVert
$$

对应四元数形式

$$
\mathrm{Exp}_q(\delta\theta)
=
\begin{bmatrix}
\cos(\theta/2)\\[1mm]
\dfrac{\delta\theta}{\theta}\sin(\theta/2)
\end{bmatrix}
$$

小角度时可近似为

$$
\mathrm{Exp}_q(\delta\theta)
\approx
\begin{bmatrix}
1\\
\frac{1}{2}\delta\theta
\end{bmatrix}
$$

---

# 2. ESKF 的名义状态与误差状态

## 2.1 名义状态

当前姿态滤波器的名义状态为

$$
\hat x = (\hat q_{nb},\ \hat b_g,\ \hat b_a)
$$

其中：

- \(\hat q_{nb}\)：body 到 nav 的姿态四元数
- \(\hat b_g\)：陀螺零偏估计
- \(\hat b_a\)：加计零偏估计

## 2.2 误差状态

误差状态定义为

$$
\delta x =
\begin{bmatrix}
\delta\theta\\
\delta b_g\\
\delta b_a
\end{bmatrix}
\in \mathbb{R}^9
$$

其中：

- \(\delta\theta\)：姿态小角误差
- \(\delta b_g\)：陀螺零偏误差
- \(\delta b_a\)：加计零偏误差

## 2.3 姿态误差定义（当前实现约定）

采用 **右乘误差定义**：

$$
C_{nb} = \hat C_{nb}\,\mathrm{Exp}(\delta\theta)
$$

这一定义非常重要，因为后续 accel update 和 mag update 的雅可比正负号都依赖它。

转置后有

$$
C_{bn} = \mathrm{Exp}(-\delta\theta)\hat C_{bn}
$$

小角度一阶展开：

$$
C_{bn} \approx (I - [\delta\theta]_\times)\hat C_{bn}
$$

## 2.4 残差定义（当前实现约定）

统一采用

$$
r = z - \hat z
$$

## 2.5 姿态注入方式（当前实现约定）

误差状态更新后，将姿态误差注入名义姿态的方式为

$$
q^+ = q^- \otimes \mathrm{Exp}_q(\delta\hat\theta)
$$

这和上面的右乘误差定义是配套的。

---

# 3. gyro predict：名义状态预测 + 协方差预测

这一部分对应函数 `algorithmESKFPredict(instance, dt)` 的数学内容。

## 3.1 陀螺测量模型

陀螺模型为

$$
\omega_m^b = \omega_{nb}^b + b_g + n_g
$$

其中：

- \(\omega_m^b\)：陀螺测量值
- \(\omega_{nb}^b\)：真实机体相对导航系的角速度，在机体系表达
- \(b_g\)：陀螺零偏
- \(n_g\)：陀螺白噪声

## 3.2 名义角速度

去偏后的名义角速度为

$$
\hat\omega^b = \omega_m^b - \hat b_g
\tag{3.2-1}
$$

## 3.3 角增量

当前采样步长为 \(dt\)，则角增量为

$$
\Delta\theta = \hat\omega^b dt
\tag{3.3-1}
$$

其模长为

$$
\theta = \lVert \Delta\theta \rVert
\tag{3.3-2}
$$

## 3.4 增量四元数

若 \(\theta \neq 0\)，则

$$
\delta q =
\mathrm{Exp}_q(\Delta\theta)
=
\begin{bmatrix}
\cos(\theta/2)\\[1mm]
\dfrac{\Delta\theta}{\theta}\sin(\theta/2)
\end{bmatrix}
\tag{3.4-1}
$$

若 \(\theta\) 很小，可用小角近似：

$$
\delta q \approx
\begin{bmatrix}
1\\
\frac{1}{2}\Delta\theta
\end{bmatrix}
\tag{3.4-2}
$$

## 3.5 姿态四元数传播

名义姿态传播为

$$
\hat q_{k+1|k} = \hat q_{k|k} \otimes \delta q
\tag{3.5-1}
$$

然后归一化：

$$
\hat q_{k+1|k} \leftarrow
\frac{\hat q_{k+1|k}}{\lVert \hat q_{k+1|k} \rVert}
\tag{3.5-2}
$$

## 3.6 bias 名义状态传播

由于 \(b_g, b_a\) 建模为随机游走，而名义状态中不显式积分噪声，所以预测时取

$$
\hat b_{g,k+1|k} = \hat b_{g,k|k}
\tag{3.6-1}
$$

$$
\hat b_{a,k+1|k} = \hat b_{a,k|k}
\tag{3.6-2}
$$

## 3.7 连续时间误差状态方程

误差状态连续模型为

$$
\delta\dot\theta
=
-[\hat\omega^b]_\times \delta\theta
- \delta b_g
- n_g
\tag{3.7-1}
$$

$$
\delta\dot b_g = n_{wg}
\tag{3.7-2}
$$

$$
\delta\dot b_a = n_{wa}
\tag{3.7-3}
$$

其中：

- \(n_g\)：陀螺白噪声
- \(n_{wg}\)：陀螺 bias 随机游走驱动噪声
- \(n_{wa}\)：加计 bias 随机游走驱动噪声

## 3.8 连续系统矩阵 \(F_c\)

将上式写成矩阵形式

$$
\delta\dot x = F_c \, \delta x + G_c w
$$

其中

$$
F_c =
\begin{bmatrix}
-[\hat\omega^b]_\times & -I_3 & 0\\
0 & 0 & 0\\
0 & 0 & 0
\end{bmatrix}
\tag{3.8-1}
$$

## 3.9 连续噪声输入矩阵 \(G_c\)

定义噪声向量

$$
w =
\begin{bmatrix}
n_g\\
n_{wg}\\
n_{wa}
\end{bmatrix}
$$

则

$$
G_c =
\begin{bmatrix}
-I_3 & 0 & 0\\
0 & I_3 & 0\\
0 & 0 & I_3
\end{bmatrix}
\tag{3.9-1}
$$

## 3.10 连续噪声强度参数

设

$$
\sigma_g = \text{gyro white noise density}
$$

$$
\sigma_{wg} = \text{gyro bias random walk density}
$$

$$
\sigma_{wa} = \text{accel bias random walk density}
$$

则连续噪声强度矩阵可写成

$$
Q_c =
\mathrm{diag}
\left(
\sigma_g^2 I_3,\;
\sigma_{wg}^2 I_3,\;
\sigma_{wa}^2 I_3
\right)
\tag{3.10-1}
$$

## 3.11 状态转移矩阵 \(\Phi\)

对 F407 上的第一版实现，一般采用一阶离散化：

$$
\Phi \approx I_9 + F_c dt
\tag{3.11-1}
$$

即

$$
\Phi =
\begin{bmatrix}
I_3 - [\hat\omega^b]_\times dt & -I_3 dt & 0\\
0 & I_3 & 0\\
0 & 0 & I_3
\end{bmatrix}
\tag{3.11-2}
$$

## 3.12 离散过程噪声协方差 \(Q_d\)

对当前这套姿态 ESKF，推荐使用如下离散 \(Q_d\)：

$$
Q_d =
\begin{bmatrix}
\left(\sigma_g^2 dt + \frac{1}{3}\sigma_{wg}^2 dt^3\right)I_3 &
-\frac{1}{2}\sigma_{wg}^2 dt^2 I_3 &
0
\\[2mm]
-\frac{1}{2}\sigma_{wg}^2 dt^2 I_3 &
\sigma_{wg}^2 dt I_3 &
0
\\[2mm]
0 & 0 &
\sigma_{wa}^2 dt I_3
\end{bmatrix}
\tag{3.12-1}
$$

定义标量

$$
q_{\theta\theta} = \sigma_g^2 dt + \frac{1}{3}\sigma_{wg}^2 dt^3
$$

$$
q_{\theta b} = -\frac{1}{2}\sigma_{wg}^2 dt^2
$$

$$
q_{bb} = \sigma_{wg}^2 dt
$$

$$
q_{ba} = \sigma_{wa}^2 dt
$$

则也可写成

$$
Q_d =
\begin{bmatrix}
q_{\theta\theta} I_3 & q_{\theta b} I_3 & 0\\
q_{\theta b} I_3 & q_{bb} I_3 & 0\\
0 & 0 & q_{ba} I_3
\end{bmatrix}
\tag{3.12-2}
$$

## 3.13 协方差预测

预测协方差为

$$
P^- = \Phi P \Phi^\top + Q_d
\tag{3.13-1}
$$

数值实现中建议做对称化：

$$
P^- \leftarrow \frac{1}{2}(P^- + P^{-\top})
\tag{3.13-2}
$$

---

# 4. accel update：利用重力方向更新 roll / pitch

这一部分对应加速度计观测更新。

## 4.1 加计观测物理意义

加计测的是 **比力**，不是“重力加速度本身”。

在 ENU 中，重力加速度向量写成

$$
g^n =
\begin{bmatrix}
0\\0\\-g
\end{bmatrix}
$$

为了方便写观测模型，引入“上向比力参考”：

$$
u^n =
\begin{bmatrix}
0\\0\\g
\end{bmatrix}
\tag{4.1-1}
$$

当线加速度可以忽略时，加计模型可写为

$$
z_a = C_{bn}u^n + b_a + n_a
\tag{4.1-2}
$$

其中：

- \(z_a\)：加计测量，机体系表达
- \(b_a\)：加计零偏
- \(n_a\)：加计测量噪声

## 4.2 当前先验下的预测加计测量

先由当前四元数得到旋转矩阵

$$
\hat C_{nb}
$$

再转置：

$$
\hat C_{bn} = \hat C_{nb}^\top
\tag{4.2-1}
$$

预测比力为

$$
\hat f^b = \hat C_{bn} u^n
\tag{4.2-2}
$$

于是预测测量为

$$
\hat z_a = \hat f^b + \hat b_a
\tag{4.2-3}
$$

## 4.3 残差定义

按当前约定

$$
r_a = z_a - \hat z_a
\tag{4.3-1}
$$

即

$$
r_a = z_a - (\hat C_{bn}u^n + \hat b_a)
\tag{4.3-2}
$$

## 4.4 accel 观测模型线性化

根据误差定义

$$
C_{bn} \approx (I - [\delta\theta]_\times)\hat C_{bn}
$$

代入观测模型：

$$
z_a = C_{bn}u^n + b_a + n_a
$$

得到

$$
z_a
\approx
(I - [\delta\theta]_\times)\hat C_{bn}u^n
+
\hat b_a + \delta b_a + n_a
$$

定义

$$
\hat f^b = \hat C_{bn}u^n
$$

则

$$
z_a
\approx
\hat f^b - [\delta\theta]_\times \hat f^b + \hat b_a + \delta b_a + n_a
$$

利用恒等式

$$
-[\delta\theta]_\times \hat f^b = [\hat f^b]_\times \delta\theta
$$

于是

$$
z_a
\approx
\hat f^b + \hat b_a + [\hat f^b]_\times \delta\theta + \delta b_a + n_a
\tag{4.4-1}
$$

因此残差满足

$$
r_a = [\hat f^b]_\times \delta\theta + \delta b_a + n_a
\tag{4.4-2}
$$

## 4.5 accel 观测雅可比 \(H_a\)

于是加计更新的观测雅可比为

$$
H_a =
\begin{bmatrix}
[\hat f^b]_\times & 0 & I_3
\end{bmatrix}
\in \mathbb{R}^{3\times 9}
\tag{4.5-1}
$$

> 注意这里是 **正号**。
>
> 这是和当前实现约定一致、并且已经通过实际验证的写法。

## 4.6 加计测量噪声协方差 \(R_a\)

设加计观测噪声协方差为

$$
R_a \in \mathbb{R}^{3\times 3}
\tag{4.6-1}
$$

常见第一版可写成

$$
R_a = \mathrm{diag}(\sigma_{ax}^2, \sigma_{ay}^2, \sigma_{az}^2)
\tag{4.6-2}
$$

或简化为

$$
R_a = \sigma_a^2 I_3
\tag{4.6-3}
$$

## 4.7 创新协方差与卡尔曼增益

$$
S_a = H_a P^- H_a^\top + R_a
\tag{4.7-1}
$$

$$
K_a = P^- H_a^\top S_a^{-1}
\tag{4.7-2}
$$

## 4.8 误差状态后验估计

$$
\delta\hat x_a = K_a r_a
\tag{4.8-1}
$$

写成分块形式：

$$
\delta\hat x_a =
\begin{bmatrix}
\delta\hat\theta_a\\
\delta\hat b_{g,a}\\
\delta\hat b_{a,a}
\end{bmatrix}
\tag{4.8-2}
$$

## 4.9 误差注入名义状态

### 姿态注入

$$
q^+ = q^- \otimes \mathrm{Exp}_q(\delta\hat\theta_a)
\tag{4.9-1}
$$

然后归一化：

$$
q^+ \leftarrow \frac{q^+}{\lVert q^+ \rVert}
\tag{4.9-2}
$$

### 陀螺 bias 注入

$$
b_g^+ = b_g^- + \delta\hat b_{g,a}
\tag{4.9-3}
$$

### 加计 bias 注入

$$
b_a^+ = b_a^- + \delta\hat b_{a,a}
\tag{4.9-4}
$$

## 4.10 Joseph 形式更新协方差

$$
P^\oplus
=
(I - K_aH_a)P^-(I - K_aH_a)^\top
+
K_a R_a K_a^\top
\tag{4.10-1}
$$

## 4.11 reset

误差注入后，要将误差坐标原点重置到新的名义状态附近。  
对当前右乘误差定义，一阶 reset 雅可比为

$$
G_{\text{reset}} =
\begin{bmatrix}
I_3 - \frac{1}{2}[\delta\hat\theta]_\times & 0 & 0\\
0 & I_3 & 0\\
0 & 0 & I_3
\end{bmatrix}
\tag{4.11-1}
$$

于是最终协方差

$$
P^+ = G_{\text{reset}} P^\oplus G_{\text{reset}}^\top
\tag{4.11-2}
$$

数值上再做对称化：

$$
P^+ \leftarrow \frac{1}{2}(P^+ + P^{+\top})
\tag{4.11-3}
$$

## 4.12 accel 更新门控

因为加计只有在“线加速度不大”时才适合当重力观测，所以通常要做门控。

### 模长门控

$$
\left|\,\lVert z_a - \hat b_a \rVert - g\,\right| < \varepsilon_a
\tag{4.12-1}
$$

### NIS 门控

$$
\gamma_a = r_a^\top S_a^{-1} r_a
\tag{4.12-2}
$$

若

$$
\gamma_a > \chi^2_{3,\alpha}
\tag{4.12-3}
$$

则拒绝本次更新。

---

# 5. yaw-only mag update：只用磁力计校正 yaw

这一部分是“只校正航向”的磁力计更新。  
它**不是**从三轴里随便挑一个分量，而是：

$$
\boxed{
\text{将参考磁向量与测量磁向量都投影到当前水平面，再比较两者的水平夹角，只用这一维角度误差来更新}
}
$$

## 5.1 磁观测的目标

当前不估计磁 bias，因此 3D 原始模型是

$$
z_m = C_{bn}h^n + n_m
\tag{5.1-1}
$$

但 yaw-only update **不直接**用这条 3D 向量方程做更新。  
而是从中提取“水平航向误差”这一维信息。

## 5.2 当前机体系中的竖直方向

由当前先验姿态得到

$$
u^b = \hat C_{bn} e_U^n
\tag{5.2-1}
$$

其中

$$
e_U^n =
\begin{bmatrix}
0\\0\\1
\end{bmatrix}
$$

## 5.3 水平投影矩阵

定义投影到“垂直于 \(u^b\) 的水平面”的投影矩阵：

$$
P_h = I_3 - u^b u^{b\top}
\tag{5.3-1}
$$

它满足：

- \(P_h u^b = 0\)
- \(P_h v\) 是 \(v\) 在当前水平面内的投影

## 5.4 参考磁向量的水平投影

设导航系中的参考磁向量为

$$
h^n
\tag{5.4-1}
$$

当前先验姿态下其在机体系中的表达为

$$
\hat h^b = \hat C_{bn} h^n
\tag{5.4-2}
$$

将其投影到当前水平面：

$$
\hat h_h^b = P_h \hat h^b
\tag{5.4-3}
$$

定义其模长

$$
\rho_h = \lVert \hat h_h^b \rVert
\tag{5.4-4}
$$

若 \(\rho_h\) 太小，则说明当前参考磁场的水平分量太弱，不适合用来校正航向。

单位化：

$$
\bar h_h^b = \frac{\hat h_h^b}{\rho_h}
\tag{5.4-5}
$$

## 5.5 测量磁向量的水平投影

测量磁向量为

$$
z_m = m^b
\tag{5.5-1}
$$

将其投影到当前水平面：

$$
m_h^b = P_h z_m
\tag{5.5-2}
$$

定义其模长：

$$
\rho_m = \lVert m_h^b \rVert
\tag{5.5-3}
$$

若 \(\rho_m\) 太小，则说明当前水平磁向量不可靠，不应更新。

单位化：

$$
\bar m_h^b = \frac{m_h^b}{\rho_m}
\tag{5.5-4}
$$

## 5.6 构造水平切向单位向量

在当前水平面内，定义与预测水平磁方向正交的单位向量

$$
s^b = u^b \times \bar h_h^b
\tag{5.6-1}
$$

这个方向表示“从预测水平磁向量逆时针转正”的切向方向（具体符号取决于当前约定）。

> 当前实现中，`algorithmESKFMagUpdateYawOnly()` 使用的是
>
> $$
> s^b = u^b \times \bar h_h^b
> $$
>
> 后续残差定义、雅可比写法、注入方向，都应与这一选择保持一致。

## 5.7 非线性 yaw 残差

定义精确的非线性标量残差为

$$
r_\psi =
\operatorname{wrap}_\pi
\left(
\operatorname{atan2}
\left(
 s^{b\top}\bar m_h^b,\;
 \bar h_h^{b\top}\bar m_h^b
\right)
\right)
\tag{5.7-1}
$$

其中 \(\operatorname{wrap}_\pi(\cdot)\) 将角度包到 \(( -\pi, \pi ]\)。

它的几何意义是：

- \(\bar h_h^b\)：当前预测的水平磁方向
- \(\bar m_h^b\)：当前测得的水平磁方向
- \(r_\psi\)：两者之间的有符号水平夹角

## 5.8 yaw-only 观测的一阶雅可比

当前实现中，yaw-only update 不是使用“简化到 \(u^{b\top}\delta\theta\)”的写法，而是直接对上面的非线性残差做一阶线性化。

对应的 \(1 \times 9\) 观测雅可比可写成

$$
H_\psi =
\begin{bmatrix}
s^{b\top}[\bar h_h^b]_\times & 0_{1\times 3} & 0_{1\times 3}
\end{bmatrix}
\tag{5.8-1}
$$

这是和当前代码实现最一致的表达。

> 说明：
>
> - 若采用另一套“更几何简写”的近似，也可写成 \(H_\psi = [u^{b\top}, 0, 0]\)
> - 但当前实现实际对应的是上面这个
>   $$
>   H_\psi = s^{b\top}[\bar h_h^b]_\times
>   $$
> - 因此文档和代码检查时，应以这一版为准。

## 5.9 yaw-only 标量测量噪声方差 \(R_\psi\)

设原始磁测量噪声协方差为

$$
R_m \in \mathbb{R}^{3\times 3}
\tag{5.9-1}
$$

则 yaw-only 残差的一阶噪声映射近似为

$$
J_{\psi m} = \frac{1}{\rho_m} s^{b\top}
\tag{5.9-2}
$$

因此更一般的标量噪声方差写法为

$$
R_\psi = J_{\psi m} R_m J_{\psi m}^\top
= \frac{1}{\rho_m^2} s^{b\top} R_m s^b
\tag{5.9-3}
$$

若 \(R_m = \sigma_m^2 I_3\)，则

$$
R_\psi = \frac{\sigma_m^2}{\rho_m^2}
\tag{5.9-4}
$$

## 5.10 创新协方差与卡尔曼增益

这是一个 1 维观测，因此

$$
S_\psi = H_\psi P^- H_\psi^\top + R_\psi
\tag{5.10-1}
$$

其中 \(S_\psi\) 是标量。

卡尔曼增益为

$$
K_\psi = P^- H_\psi^\top S_\psi^{-1}
\tag{5.10-2}
$$

维度：

$$
K_\psi \in \mathbb{R}^{9\times 1}
$$

## 5.11 误差状态后验估计

$$
\delta\hat x_\psi = K_\psi r_\psi
\tag{5.11-1}
$$

写成分块形式：

$$
\delta\hat x_\psi =
\begin{bmatrix}
\delta\hat\theta_\psi\\
\delta\hat b_{g,\psi}\\
\delta\hat b_{a,\psi}
\end{bmatrix}
\tag{5.11-2}
$$

## 5.12 误差注入名义状态

### 姿态注入

$$
q^+ = q^- \otimes \mathrm{Exp}_q(\delta\hat\theta_\psi)
\tag{5.12-1}
$$

归一化：

$$
q^+ \leftarrow \frac{q^+}{\lVert q^+ \rVert}
\tag{5.12-2}
$$

### 陀螺 bias 注入

$$
b_g^+ = b_g^- + \delta\hat b_{g,\psi}
\tag{5.12-3}
$$

### 加计 bias 注入

$$
b_a^+ = b_a^- + \delta\hat b_{a,\psi}
\tag{5.12-4}
$$

## 5.13 Joseph 形式更新协方差

$$
P^\oplus =
(I - K_\psi H_\psi)P^-(I - K_\psi H_\psi)^\top
+
K_\psi R_\psi K_\psi^\top
\tag{5.13-1}
$$

## 5.14 reset

误差注入后仍然要做 reset：

$$
G_{\text{reset}} =
\begin{bmatrix}
I_3 - \frac{1}{2}[\delta\hat\theta]_\times & 0 & 0\\
0 & I_3 & 0\\
0 & 0 & I_3
\end{bmatrix}
\tag{5.14-1}
$$

最终协方差：

$$
P^+ = G_{\text{reset}} P^\oplus G_{\text{reset}}^\top
\tag{5.14-2}
$$

并做对称化：

$$
P^+ \leftarrow \frac{1}{2}(P^+ + P^{+\top})
\tag{5.14-3}
$$

## 5.15 yaw-only mag update 门控

### 水平参考向量门控

若水平参考磁向量过小，则不更新：

$$
\rho_h < \varepsilon_h
\tag{5.15-1}
$$

### 水平测量向量门控

若水平测量磁向量过小，则不更新：

$$
\rho_m < \varepsilon_m
\tag{5.15-2}
$$

### 磁场模长门控

若原始磁场模长异常，则拒绝更新：

$$
\big|\lVert z_m \rVert - H_0\big| > \varepsilon_H
\tag{5.15-3}
$$

### NIS 门控

$$
\gamma_\psi = \frac{r_\psi^2}{S_\psi}
\tag{5.15-4}
$$

若

$$
\gamma_\psi > \chi^2_{1,\alpha}
\tag{5.15-5}
$$

则拒绝本次更新。

---

# 6. 这一整套 ESKF 的时间顺序

当前姿态滤波器的推荐运行顺序为：

## 每个 IMU 周期

1. 读取 gyro + accel
2. 执行 gyro predict
3. accel 通过门控时，执行 accel update

## 每个 mag 新样本到来时

1. 读取磁测量
2. 若通过门控，执行 yaw-only mag update

可写成：

$$
\boxed{
\text{gyro predict} \;\rightarrow\; \text{accel update} \;\rightarrow\; (\text{有 mag 时})\ \text{yaw-only mag update}
}
$$

---

# 7. 这一套实现中最容易出错的地方

## 7.1 姿态误差定义与雅可比符号不一致

当前这套文档统一采用：

$$
C_{nb} = \hat C_{nb}\mathrm{Exp}(\delta\theta)
$$

$$
r = z - \hat z
$$

$$
q^+ = q^- \otimes \mathrm{Exp}_q(\delta\hat\theta)
$$

在这个约定下，accel update 的雅可比必须是

$$
H_a =
\begin{bmatrix}
[\hat f^b]_\times & 0 & I_3
\end{bmatrix}
$$

而不是带负号的版本。

## 7.2 将 yaw-only 的“几何残差”与“简化雅可比”混用

当前实现的 yaw-only mag update 采用的是：

- 非线性残差：
  $$
  r_\psi = \operatorname{atan2}(s^\top \bar m_h,\ \bar h_h^\top \bar m_h)
  $$
- 线性化雅可比：
  $$
  H_\psi = s^\top[\bar h_h^b]_\times
  $$

因此文档和代码检查时应以这一版为准，不要再与别的近似写法硬混。

## 7.3 磁向量未完成标定 / 坐标对齐

yaw-only mag update 的前提是：

- 已完成硬铁补偿
- 已完成软铁补偿
- 已旋转到机体系 FLU

否则水平投影后的磁方向本身就是错的。

## 7.4 将“磁航向”误当“真北航向”

若参考磁向量 \(h^n\) 未补磁偏角 \(D\)，则 yaw-only 磁更新得到的是 **磁航向**，不是真北航向。

---

# 8. 复习时建议记住的最核心公式

如果只记最关键的，建议记下面这组。

## gyro predict

$$
\hat\omega^b = \omega_m^b - \hat b_g
$$

$$
\Delta\theta = \hat\omega^b dt
$$

$$
q^- = q \otimes \mathrm{Exp}_q(\Delta\theta)
$$

$$
P^- = \Phi P \Phi^\top + Q_d
$$

## accel update

$$
\hat f^b = \hat C_{bn}u^n
$$

$$
r_a = z_a - (\hat f^b + \hat b_a)
$$

$$
H_a =
\begin{bmatrix}
[\hat f^b]_\times & 0 & I_3
\end{bmatrix}
$$

## yaw-only mag update

$$
u^b = \hat C_{bn}e_U^n,\qquad P_h = I - u^b u^{b\top}
$$

$$
\bar h_h^b = \frac{P_h \hat C_{bn}h^n}{\lVert P_h \hat C_{bn}h^n \rVert},
\qquad
\bar m_h^b = \frac{P_h z_m}{\lVert P_h z_m \rVert}
$$

$$
s^b = u^b \times \bar h_h^b
$$

$$
r_\psi =
\operatorname{wrap}_\pi
\left(
\operatorname{atan2}
\left(
 s^{b\top}\bar m_h^b,\;
 \bar h_h^{b\top}\bar m_h^b
\right)
\right)
$$

$$
H_\psi =
\begin{bmatrix}
s^{b\top}[\bar h_h^b]_\times & 0 & 0
\end{bmatrix}
$$

---

# 9. 总结

这套姿态 ESKF 的职责分工可以概括为：

- **gyro predict**：负责短时间连续传播姿态
- **accel update**：利用重力方向稳定 roll / pitch
- **yaw-only mag update**：利用水平磁方向只校正 yaw

因此：

$$
\boxed{
\text{gyro 管“怎么转过去”，accel 管“倾斜对不对”，yaw-only mag 管“朝向对不对”}
}
$$

这就是当前这一版 ESKF 的完整数学主线。
