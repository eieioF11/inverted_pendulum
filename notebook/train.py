import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
from matplotlib import animation

class SquattingPendulumEnv(gym.Env):
    """
    屈伸型倒立振子のカスタム環境
    アクション: 振子の長さの加速度（または伸縮速度の変化）
    観測: 角度(cos, sin), 角速度, 長さ, 伸縮速度
    """
    def __init__(self):
        super(SquattingPendulumEnv, self).__init__()
        
        # 物理定数
        self.g = 9.81
        self.m = 1.0  # 質量 (kg)
        self.dt = 0.05 # 時間刻み
        
        # 制約
        self.l_min = 0.5 # 最小長さ (m)
        self.l_max = 1.5 # 最大長さ (m)
        self.max_speed = 8.0
        
        # アクション空間: 長さの加速度 (ddot_r)
        # -1.0 〜 1.0 の範囲で正規化し、内部でスケーリングします
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        
        # 観測空間: [cos(theta), sin(theta), theta_dot, r, r_dot]
        high = np.array([1.0, 1.0, np.inf, self.l_max, np.inf], dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)
        
        self.state = None
        self.steps_beyond_done = None

    def step(self, action):
        # アクションの適用（長さの加速度）
        r_acc = np.clip(action[0], -1.0, 1.0) * 5.0 # 加速度のスケール調整
        
        theta, theta_dot, r, r_dot = self.state
        
        # 運動方程式 (ラグランジュ方程式より導出)
        # d/dt(mr^2 * theta_dot) - mgr*sin(theta) = 0 (自由回転ピボットの場合)
        # => mr^2 * theta_ddot + 2mr*r_dot*theta_dot - mgr*sin(theta) = 0
        # => theta_ddot = (g * sin(theta) - 2 * r_dot * theta_dot) / r
        
        # 倒立状態(theta=0)を維持したいので、重力項の符号に注意（座標系による）
        # ここでは theta=0 を真上、時計回りを正とします。
        # 復元力を得るための運動方程式
        theta_acc = (self.g * np.sin(theta) - 2 * r_dot * theta_dot) / r
        
        # オイラー法による積分 (簡易シミュレーション)
        theta_dot_new = theta_dot + theta_acc * self.dt
        theta_new = theta + theta_dot_new * self.dt
        
        r_dot_new = r_dot + r_acc * self.dt
        r_new = r + r_dot_new * self.dt
        
        # 摩擦減衰（現実的な挙動のため少し入れる）
        theta_dot_new *= 0.999 
        
        # 長さの物理的制約（衝突）
        if r_new < self.l_min:
            r_new = self.l_min
            r_dot_new = 0
        elif r_new > self.l_max:
            r_new = self.l_max
            r_dot_new = 0
            
        self.state = np.array([theta_new, theta_dot_new, r_new, r_dot_new])
        
        # 角度の正規化 (-pi to pi)
        theta_norm = (theta_new + np.pi) % (2 * np.pi) - np.pi
        
        # 終了判定: 倒れすぎた場合 (±45度以上など)
        terminated = bool(abs(theta_norm) > 1.0) # 約57度
        if r_new < self.l_min or r_new > self.l_max:
            # 長さ制約違反はペナルティだが即終了ではない（クリップしてるので）
            pass

        # 報酬関数の設計
        # 1. 角度が0に近いほど高報酬
        # 2. 角速度が小さいほど高報酬（安定）
        # 3. アクション（急激な伸縮）は少なめに（省エネ）
        reward = 1.0 - theta_norm**2 - 0.1 * (theta_dot_new**2) - 0.01 * (action[0]**2)
        
        if terminated:
            reward = -10.0 # 転倒ペナルティ
        
        # 観測の作成
        obs = np.array([np.cos(theta_new), np.sin(theta_new), theta_dot_new, r_new, r_dot_new], dtype=np.float32)
        
        return obs, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 初期状態: ほぼ真上だが、わずかにランダムに傾いている状態からスタート
        theta_init = np.random.uniform(-0.1, 0.1)
        r_init = np.random.uniform(0.8, 1.2)
        self.state = np.array([theta_init, 0.0, r_init, 0.0])
        
        obs = np.array([np.cos(theta_init), np.sin(theta_init), 0.0, r_init, 0.0], dtype=np.float32)
        return obs, {}

    def render(self):
        # 可視化用（簡易テキスト出力、またはMatplotlib用のデータを返すなど）
        pass

# --- メイン処理 ---

# 1. 環境の生成
env = SquattingPendulumEnv()

# 2. モデルの定義 (PPO)
model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.0003)

print("学習を開始します...")
# 3. 学習の実行 (ステップ数は調整してください)
model.learn(total_timesteps=50000)
print("学習完了！")

# 4. テストと評価
print("テスト実行中...")
obs, _ = env.reset()
theta_history = []
r_history = []

for _ in range(300):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    
    # ログ保存（観測データのインデックスを元に戻して取得）
    # obs: [cos, sin, theta_dot, r, r_dot]
    # thetaはcos, sinから逆算するか、env.stateにアクセス
    theta_current = np.arctan2(obs[1], obs[0])
    theta_history.append(theta_current)
    r_history.append(obs[3])
    
    if done:
        obs, _ = env.reset()

# 5. 結果のプロット
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(theta_history)
plt.title("Pendulum Angle (Theta)")
plt.ylabel("Angle [rad]")
plt.axhline(0, color='r', linestyle='--') # 目標値

plt.subplot(2, 1, 2)
plt.plot(r_history, color='orange')
plt.title("Pendulum Length (r) - Squatting Action")
plt.ylabel("Length [m]")
plt.xlabel("Time Step")

plt.tight_layout()
plt.show()