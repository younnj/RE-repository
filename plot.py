import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os

# --- 1. 설정 및 메타데이터 ---

# 3가지 제어 방식의 로그 파일 이름. (시뮬레이션 후 생성된 파일 이름과 일치시켜야 함)
LOG_FILES = {
    'Jacobian (Open-Loop)': 'log_jac.txt',
    'CLIK':                 'log_clik.txt',
    'WPI CLIK':             'log_wpi.txt',
}

# 로깅된 데이터의 컬럼 이름 정의 (총 14개)
# Time(1) | x_desired(3) | x_current(3) | q_desired(7)
COLUMNS = (
    ['Time'] + 
    [f'xd{i}' for i in range(1, 4)] + 
    [f'xc{i}' for i in range(1, 4)] + 
    [f'qd{i}' for i in range(1, 8)]
)

# 튜닝된 Kp 게인 (문제 2/3에서 사용했을 값으로 대체)
KP_VALUE = 5.0 

# --- 2. 데이터 로딩 및 정규화 함수 ---
def load_and_normalize_data():
    """로그 파일들을 읽고 시간을 t=0부터 시작하도록 정규화"""
    data_frames = {}
    
    for method, filename in LOG_FILES.items():
        if not os.path.exists(filename):
            print(f"경고: 파일 '{filename}'을 찾을 수 없습니다. 모드 ({method})는 건너뜁니다.")
            continue
            
        try:
            # Raw String (r'\s+')을 사용하여 공백을 구분자로 정확히 인식
            df = pd.read_csv(filename, sep=r'\s+', header=None)
            df.columns = COLUMNS[:len(df.columns)]
            
            # 1. 시간 정규화 (Normalization): 첫 번째 레코드의 시간을 0으로 맞춤
            start_time = df['Time'].min()
            df['Time'] = df['Time'] - start_time
            
            # 2. 궤적 지속 시간 (2.0초) 이후 데이터 자르기 (옵션)
            # Cubic Trajectory가 끝난 후의 잔여 로그를 제거하여 비교를 명확하게 함
            df = df[df['Time'] <= 2.05] 
            
            data_frames[method] = df
            print(f"데이터 로드 성공: {method} ({df.shape[0]} 행, Time: {df['Time'].max():.3f}s)")
        except Exception as e:
            print(f"오류: {filename} 파일을 읽는 중 오류 발생: {e}")
            continue
            
    return data_frames

# --- 3. Task Space 3D 플롯 함수 ---
def plot_3d_trajectories(data_frames):
    """원하는 궤적 (1개)과 3가지 제어 방식의 현재 궤적 (3개)을 3D로 비교"""
    if not data_frames: return

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 색상 및 스타일 정의
    styles = {
        'Jacobian (Open-Loop)': ('red', '--'), 
        'CLIK': ('blue', '-'), 
        'WPI CLIK': ('green', '-'),
    }
    
    # 1. Desired Trajectory (모든 방법에서 동일하므로 하나만 플롯)
    first_method = list(data_frames.keys())[0]
    df_desired = data_frames[first_method]
    ax.plot(df_desired['xd1'], df_desired['xd2'], df_desired['xd3'], 
            label='Desired Trajectory', color='k', linestyle=':', linewidth=3)
    
    # 2. Current Trajectories (3가지 제어 방식 모두 플롯)
    for method, df in data_frames.items():
        color, linestyle = styles[method]
        ax.plot(df['xc1'], df['xc2'], df['xc3'], 
                label=f'Current ({method})', color=color, linestyle=linestyle, linewidth=1.5)

    ax.set_title('End-Effector Trajectory Comparison in 3D Space (Kp={})'.format(KP_VALUE))
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.legend(loc='best', title='Trajectory')
    ax.grid(True)
    
    # 플롯 범위 설정 (모든 데이터가 보이도록)
    all_x = pd.concat([df['xd1'] for df in data_frames.values()] + [df['xc1'] for df in data_frames.values()])
    all_y = pd.concat([df['xd2'] for df in data_frames.values()] + [df['xc2'] for df in data_frames.values()])
    all_z = pd.concat([df['xd3'] for df in data_frames.values()] + [df['xc3'] for df in data_frames.values()])
    
    ax.set_xlim(all_x.min(), all_x.max())
    ax.set_ylim(all_y.min(), all_y.max())
    ax.set_zlim(all_z.min(), all_z.max())
    
    plt.show()

# --- 4. Joint Space 2D 플롯 함수 ---
def plot_joint_trajectories(data_frames):
    """3가지 제어 방식의 q_desired를 7개의 서브플롯으로 분리하여 비교"""
    if not data_frames: return
        
    # 7개의 관절에 대해 4x2 서브플롯 구성
    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    axes = axes.flatten()[:7] # 8번째(3, 1)는 사용하지 않음
    fig.suptitle('Desired Joint Trajectories Comparison (q_desired)', fontsize=16)

    # 색상 및 스타일 정의
    styles = {'Jacobian (Open-Loop)': 'r--', 'CLIK': 'b-', 'WPI CLIK': 'g-'}

    for i in range(7):
        joint_col = f'qd{i+1}'
        ax = axes[i]
        
        # 3가지 방법의 q_desired를 플롯
        for method, df in data_frames.items():
            ax.plot(df['Time'], df[joint_col], 
                    label=method, 
                    linestyle=styles[method][1:], 
                    color=styles[method][0],
                    linewidth=1.5)

        ax.set_title(f'Joint {i+1} (q{i+1})')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angle (rad)')
        ax.grid(True)
        
        # 범례는 첫 번째 플롯에만 표시
        if i == 0:
            ax.legend(loc='best', fontsize='small')

    # 마지막 빈 서브플롯 제거
    fig.delaxes(axes[-1])
    
    plt.tight_layout(rect=[0, 0, 1, 0.96]) # 타이틀 공간 확보
    plt.show()

# --- 5. 메인 실행 ---
if __name__ == '__main__':
    data_frames = load_and_normalize_data()
    
    if data_frames:
        print("\n--- 시각화 시작 ---")
        # 3D Task Space 플롯
        plot_3d_trajectories(data_frames)
        
        # 2D Joint Space 플롯
        plot_joint_trajectories(data_frames)
    else:
        print("\n시각화를 위해 로그 파일 3개 중 최소한 하나가 올바른 경로에 있는지 확인해주세요.")