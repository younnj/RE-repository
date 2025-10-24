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

# --- 2. 데이터 로딩 및 정규화 함수 (수정됨: 패딩 로직 추가) ---
def load_and_normalize_data(max_duration=None):
    """로그 파일들을 읽고 시간을 t=0부터 시작하도록 정규화하며, 가장 긴 데이터에 맞춰 패딩"""
    raw_data_frames = {}
    
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
            df['Time_norm'] = df['Time'] - start_time
            
            # 2. 궤적 지속 시간 (2.0초) 이후 데이터 자르기 (이전 문제 해결을 위해 주석 처리)
            # df = df[df['Time_norm'] <= 2.05] 

            # 3. 최대 시간 제한 (선택 사항): max_duration이 설정되면 그 이후는 자름
            if max_duration is not None:
                 df = df[df['Time_norm'] <= max_duration]
            
            raw_data_frames[method] = df.drop(columns=['Time']).rename(columns={'Time_norm': 'Time'}) # Time_norm을 Time으로 대체
            print(f"데이터 로드 성공: {method} ({df.shape[0]} 행, Time: {df['Time_norm'].max():.3f}s)")
        except Exception as e:
            print(f"오류: {filename} 파일을 읽는 중 오류 발생: {e}")
            continue
            
    if not raw_data_frames:
        return {}

    # --- 4. 패딩 로직 시작 ---
    # 가장 긴 데이터프레임의 길이를 찾습니다.
    max_len = max(len(df) for df in raw_data_frames.values())
    data_frames = {}
    
    for method, df in raw_data_frames.items():
        current_len = len(df)
        
        if current_len < max_len:
            print(f"패딩: {method} ({current_len} -> {max_len})")
            
            # 패딩해야 할 행의 개수
            padding_rows_count = max_len - current_len
            
            # 마지막 행을 추출
            last_row = df.iloc[-1].copy()
            
            # 마지막 Time 컬럼 값을 최대 Time 값으로 설정
            # 패딩된 시간은 가장 긴 데이터의 마지막 시간과 일치하도록 조정 (선형 증가)
            max_time = max(raw_df['Time'].max() for raw_df in raw_data_frames.values())
            time_diff = max_time - df['Time'].iloc[-1]
            time_step = time_diff / padding_rows_count if padding_rows_count > 0 else 0
            
            # 패딩 데이터프레임 생성
            padding_data = {col: [last_row[col]] * padding_rows_count for col in df.columns}
            df_padding = pd.DataFrame(padding_data)
            
            # Time 컬럼은 마지막 시간부터 max_time까지 선형적으로 증가
            df_padding['Time'] = np.linspace(df['Time'].iloc[-1] + time_step, max_time, padding_rows_count)
            
            # 원래 데이터와 패딩 데이터를 결합
            df_padded = pd.concat([df, df_padding], ignore_index=True)
            data_frames[method] = df_padded
        else:
            data_frames[method] = df
            
    return data_frames

# --- A. 좌표 포맷팅 헬퍼 함수 ---
def format_coords(x, y, z):
    """좌표를 (x.xxx, y.xxx, z.xxx) 형식으로 포맷팅"""
    return f"({x:.3f}, {y:.3f}, {z:.3f})"


# --- 3. Task Space 3D 플롯 함수 (수정 없음) ---
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
    
    # 목표 궤적은 모든 방법에서 'xd'가 동일해야 하므로 첫 번째 데이터프레임 사용
    # 단, 패딩된 데이터는 'xd'도 패딩되어 있으므로, 궤적의 변화가 끝난 시점까지만 사용
    xd_change_idx = df_desired[df_desired['xd1'] != df_desired['xd1'].iloc[-1]].index.max()
    xd_change_idx = xd_change_idx if pd.notna(xd_change_idx) else len(df_desired) - 1
    
    df_plot_xd = df_desired.iloc[:xd_change_idx + 2] # 궤적 끝난 후 한두 포인트 포함
    
    ax.plot(df_plot_xd['xd1'], df_plot_xd['xd2'], df_plot_xd['xd3'], 
            label='Desired Trajectory', color='k', linestyle=':', linewidth=3)
    
    # **Desired Trajectory의 목표 지점 좌표 표시 (마지막 값이 아닌 목표 값)**
    xd_target_coords = df_plot_xd.iloc[-1][['xd1', 'xd2', 'xd3']].values
    ax.text(xd_target_coords[0], xd_target_coords[1], xd_target_coords[2], 
            f'Desired Target: {format_coords(*xd_target_coords)}', 
            color='k', fontsize=10, ha='center', va='bottom')
    
    # 2. Current Trajectories (3가지 제어 방식 모두 플롯)
    for method, df in data_frames.items():
        color, linestyle = styles[method]
        ax.plot(df['xc1'], df['xc2'], df['xc3'], 
                label=f'Current ({method})', color=color, linestyle=linestyle, linewidth=1.5)
        
        # **Current Trajectory의 최종 지점 좌표 표시 (패딩된 최종 값)**
        last_xc = df.iloc[-1]
        xc_coords = last_xc[['xc1', 'xc2', 'xc3']].values
        ax.text(xc_coords[0], xc_coords[1], xc_coords[2], 
                f'{method} End: {format_coords(*xc_coords)}', 
                color=color, fontsize=9, ha='center', va='bottom')


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

# --- 4. Joint Space 2D 플롯 함수 (수정됨: Time 컬럼 사용) ---
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
    # 최대 시간(초)을 인수로 전달하여 Open-Loop의 발산이 너무 길어지는 것을 방지할 수 있습니다. 
    # 예를 들어, CLIK이 27초에 끝났다면 max_duration=30.0을 사용하거나 None을 사용하여 가장 긴 길이만큼 맞춥니다.
    data_frames = load_and_normalize_data(max_duration=None)
    
    if data_frames:
        print("\n--- 시각화 시작 ---")
        # 3D Task Space 플롯
        plot_3d_trajectories(data_frames)
        
        # 2D Joint Space 플롯
        plot_joint_trajectories(data_frames)
    else:
        print("\n시각화를 위해 로그 파일 3개 중 최소한 하나가 올바른 경로에 있는지 확인해주세요.")