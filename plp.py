import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.envs import DummyVecEnv

# パラレルリンク倒立振子の環境クラス


class ParallelLinkPendulumEnv(gym.Env):
    def __init__(self):
        super(ParallelLinkPendulumEnv, self).__init__()
        self.dt = 0.02  # 時間ステップ
        self.max_time = 5  # 最大シミュレーション時間
        self.time = 0

        # 状態と行動の空間
        self.observation_space = gym.spaces.Box(
            low=-np.pi, high=np.pi, shape=(4,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-10, high=10, shape=(2,), dtype=np.float32)

        self.reset()

    def equations(self, t, state, u1, u2):
        theta1, omega1, theta2, omega2 = state
        domega1_dt = (u1 - m1*g*l1*np.sin(theta1)) / I1
        domega2_dt = (u2 - m2*g*l2*np.sin(theta2)) / I2
        return [omega1, domega1_dt, omega2, domega2_dt]

    def step(self, action):
        u1, u2 = action
        sol = solve_ivp(self.equations, [0, self.dt], self.state, args=(
            u1, u2), t_eval=[self.dt])
        self.state = sol.y[:, -1]
        self.time += self.dt

        # 目標状態: (theta1, theta2) ≈ (0,0)
        reward = - (self.state[0]**2 + self.state[2]**2)
        done = self.time >= self.max_time

        return np.array(self.state, dtype=np.float32), reward, done, {}

    def reset(self):
        self.state = np.random.uniform(-0.1, 0.1, size=(4,))
        self.time = 0
        return np.array(self.state, dtype=np.float32)


# 環境の作成
env = DummyVecEnv([lambda: ParallelLinkPendulumEnv()])

# 強化学習モデルの学習
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)

# 学習済みモデルのテスト
done = False
state = env.reset()
while not done:
    action, _ = model.predict(state)
    state, reward, done, _ = env.step(action)
    print(f"State: {state}, Reward: {reward}")
