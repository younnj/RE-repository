import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os

# --- 1. 설정 및 메타데이터 ---

LOG_FILES = {
    'Combined Jacobian (Problem 1)': 'log_2tasks.txt',
    'Nullspace (Problem 2)': 'log_nullspace.txt',
    'Task Transition (Problem 3)': 'log_tasktrans.txt',
}

# --- 2. 데이터 로딩 및 정규화 함수 ---
def load_and_normalize_data(max_duration=None):
    """로그 파일들을 읽고 시간을 t=0부터 시작하도록 정규화하며, 가장 긴 데이터에 맞춰 패딩"""
    raw_data_frames = {}
    
    # 기본 컬럼 정의 (h2 제외, 20개)
    BASE_COLUMNS = (
        ['Time'] + 
        [f'xd{i}' for i in range(1, 7)] +
        [f'xc{i}' for i in range(1, 7)] +
        [f'qd{i}' for i in range(1, 8)]
    )

    for method, filename in LOG_FILES.items():
        if not os.path.exists(filename):
            print(f"경고: 파일 '{filename}'을 찾을 수 없습니다. 모드 ({method})는 건너뜁니다.")
            continue
            
        try:
            df = pd.read_csv(filename, sep=r'\s+', header=None)
            df = df.dropna(axis=1, how='all') # 빈 컬럼 제거
            
            current_cols = BASE_COLUMNS[:]
            
            # Task Transition 파일인 경우에만 h2 컬럼을 추가 (21개 컬럼)
            if method == 'Task Transition (Problem 3)':
                current_cols.append('h2')

            if len(df.columns) < len(current_cols):
                 print(f"경고: {method} 파일의 데이터 열이 부족합니다 ({len(df.columns)}/{len(current_cols)}). h2 로깅을 확인하세요.")
                 if 'h2' in current_cols:
                      current_cols.pop() 
            
            df.columns = current_cols[:len(df.columns)]
            
            start_time = df['Time'].min()
            df['Time_norm'] = df['Time'] - start_time
            
            if max_duration is not None:
                 df = df[df['Time_norm'] <= max_duration]
            
            raw_data_frames[method] = df.drop(columns=['Time']).rename(columns={'Time_norm': 'Time'})
            print(f"데이터 로드 성공: {method} ({df.shape[0]} 행, Time: {df['Time_norm'].max():.3f}s)")
        except Exception as e:
            print(f"오류: {filename} 파일을 읽는 중 오류 발생: {e}")
            continue
            
    if not raw_data_frames:
        return {}
    
    # --- 3. 패딩 로직 ---
    max_len = max(len(df) for df in raw_data_frames.values())
    data_frames = {}
    
    for method, df in raw_data_frames.items():
        current_len = len(df)
        
        if current_len < max_len:
            padding_rows_count = max_len - current_len
            last_row = df.iloc[-1].copy()
            max_time = max(raw_df['Time'].max() for raw_df in raw_data_frames.values())
            time_diff = max_time - df['Time'].iloc[-1]
            time_step = time_diff / padding_rows_count if padding_rows_count > 0 else 0
            
            padding_data = {col: [last_row[col]] * padding_rows_count for col in df.columns}
            df_padding = pd.DataFrame(padding_data)
            df_padding['Time'] = np.linspace(df['Time'].iloc[-1] + time_step, max_time, padding_rows_count)
            
            df_padded = pd.concat([df, df_padding], ignore_index=True)
            data_frames[method] = df_padded
        else:
            data_frames[method] = df
            
    return data_frames


# --- A. 좌표 포맷팅 헬퍼 함수 ---
def format_coords(x, y, z):
    """좌표를 (x.xxx, y.xxx, z.xxx) 형식으로 포맷팅"""
    return f"({x:.3f}, {y:.3f}, {z:.3f})"

# --- 3. Task 1 (EE) 3D 플롯 함수 ---
def plot_3d_ee_trajectories(data_frames):
    """End-Effector (x1, xc1~3)의 궤적을 3D로 비교"""
    if not data_frames: return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    styles = {
        'Combined Jacobian (Problem 1)': 'r-', 
        'Nullspace (Problem 2)': 'b-', 
        'Task Transition (Problem 3)': 'g-',
    }
    
    first_method = list(data_frames.keys())[0]
    df_desired = data_frames[first_method]
    
    # --- Desired Trajectory (EE) ---
    # xd 값의 변화가 멈춘 인덱스를 찾기 (궤적 종료 시점)
    xd_cols = ['xd1', 'xd2', 'xd3']
    xd_stable_idx = df_desired[xd_cols].apply(lambda x: x.iloc[-1] != x).any(axis=1).idxmax()
    xd_stable_idx = xd_stable_idx if pd.notna(xd_stable_idx) else len(df_desired) # 안정화 지점
    
    # 궤적 플롯은 안정화 지점까지만 사용 (+10은 마진)
    df_plot_xd = df_desired.iloc[:xd_stable_idx + 10]
    ax.plot(df_plot_xd['xd1'], df_plot_xd['xd2'], df_plot_xd['xd3'], 
            label='Desired EE Trajectory', color='k', linestyle=':', linewidth=3)
    
    # --- Desired Target (EE) ---
    # **수정된 로직:** 로그 파일의 마지막 값 (궤적이 목표에 고정된 값)을 사용
    final_xd_row = df_desired.iloc[-1]
    xd_target_coords = final_xd_row[['xd1', 'xd2', 'xd3']].values
    
    ax.scatter(xd_target_coords[0], xd_target_coords[1], xd_target_coords[2], 
               color='k', marker='o', s=50)
    ax.text(xd_target_coords[0], xd_target_coords[1], xd_target_coords[2], 
            f'Target (EE): {format_coords(*xd_target_coords)}', 
            color='k', fontsize=10, ha='center', va='bottom')
    
    # Current Trajectories (EE)
    for method, df in data_frames.items():
        color, linestyle = styles[method]
        ax.plot(df['xc1'], df['xc2'], df['xc3'], 
                label=f'Current ({method})', color=color[0], linestyle=linestyle, linewidth=1.5)
        
        # Final Point
        last_xc = df.iloc[-1]
        xc_coords = last_xc[['xc1', 'xc2', 'xc3']].values
        ax.scatter(xc_coords[0], xc_coords[1], xc_coords[2], color=color[0], marker='x', s=50)


    ax.set_title('Task 1: End-Effector Position Tracking (x1)')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.legend(loc='best', title='Control Method')
    ax.grid(True)
    plt.show()

# --- 4. Task 2 (Link 4) 3D 플롯 함수 ---
def plot_3d_link4_trajectories(data_frames):
    """Link 4 (x2, xc4~6)의 궤적을 3D로 비교"""
    if not data_frames: return

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    styles = {
        'Combined Jacobian (Problem 1)': 'r-', 
        'Nullspace (Problem 2)': 'b-', 
        'Task Transition (Problem 3)': 'g-',
    }
    
    first_method = list(data_frames.keys())[0]
    df_desired = data_frames[first_method]
    
    # Desired Trajectory (Link 4)
    xd_cols = ['xd4', 'xd5', 'xd6']
    xd_stable_idx = df_desired[xd_cols].apply(lambda x: x.iloc[-1] != x).any(axis=1).idxmax()
    xd_stable_idx = xd_stable_idx if pd.notna(xd_stable_idx) else len(df_desired)

    df_plot_xd = df_desired.iloc[:xd_stable_idx + 10]
    ax.plot(df_plot_xd['xd4'], df_plot_xd['xd5'], df_plot_xd['xd6'], 
            label='Desired Link 4 Trajectory', color='k', linestyle=':', linewidth=3)
    
    # Desired Target (Link 4)
    # **수정된 로직:** 로그 파일의 마지막 값 (궤적이 목표에 고정된 값)을 사용
    final_xd_row = df_desired.iloc[-1]
    xd_target_coords = final_xd_row[['xd4', 'xd5', 'xd6']].values
    
    ax.scatter(xd_target_coords[0], xd_target_coords[1], xd_target_coords[2], 
               color='k', marker='o', s=50)
    ax.text(xd_target_coords[0], xd_target_coords[1], xd_target_coords[2], 
            f'Target (Link 4): {format_coords(*xd_target_coords)}', 
            color='k', fontsize=10, ha='center', va='bottom')
    
    # Current Trajectories (Link 4)
    for method, df in data_frames.items():
        color, linestyle = styles[method]
        ax.plot(df['xc4'], df['xc5'], df['xc6'], 
                label=f'Current ({method})', color=color[0], linestyle=linestyle, linewidth=1.5)
        
        # Final Point
        last_xc = df.iloc[-1]
        xc_coords = last_xc[['xc4', 'xc5', 'xc6']].values
        ax.scatter(xc_coords[0], xc_coords[1], xc_coords[2], color=color[0], marker='x', s=50)


    ax.set_title('Task 2: Link 4 Position Tracking (x2)')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.legend(loc='best', title='Control Method')
    ax.grid(True)
    plt.show()

# --- 5. H2 Activation Parameter 플롯 함수 ---
def plot_h2_activation(data_frames):
    """Task Transition (Problem 3)의 h2 활성화 파라미터를 플롯"""
    if 'Task Transition (Problem 3)' not in data_frames:
        print("경고: Task Transition 데이터가 없거나 h2 컬럼이 누락되어 h2 플롯을 건너뜁니다.")
        return

    df = data_frames['Task Transition (Problem 3)']
    
    if 'h2' not in df.columns:
         print("경고: Task Transition 데이터에 'h2' 컬럼이 누락되어 h2 플롯을 건너뜁니다.")
         return
         
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(df['Time'], df['h2'], label='h2 Activation Parameter', color='purple', linewidth=2)
    ax.set_title('Task 2 Activation Parameter ($h_2$) over Time (Task Transition)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('$h_2$ Value (0 to 1)')
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True)
    ax.legend()
    plt.show()


# --- 6. Joint Space 2D 플롯 함수 (수정 없음) ---
def plot_joint_trajectories(data_frames):
    """3가지 제어 방식의 q_desired를 7개의 서브플롯으로 분리하여 비교"""
    if not data_frames: return
        
    # 4x2 그리드 생성 (총 8개, 7개 사용)
    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    axes = axes.flatten()[:7] # Joint 1 ~ Joint 7의 서브플롯만 남김
    fig.suptitle('Desired Joint Trajectories Comparison (q_desired)', fontsize=16)

    styles = {
        'Combined Jacobian (Problem 1)': 'r-', 
        'Nullspace (Problem 2)': 'b-', 
        'Task Transition (Problem 3)': 'g-',
    }

    for i in range(7):
        joint_col = f'qd{i+1}'
        ax = axes[i]
        
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
        
        if i == 0:
            ax.legend(loc='best', fontsize='small')

    # fig.delaxes(axes[-1])  <-- 이 라인이 Joint 7의 플롯을 삭제했으므로 제거!
    
    # 8번째 (빈) 서브플롯이 남아있다면 삭제해야 합니다.
    # axes 배열은 이미 7개이므로, 8번째 축 객체(axes[7])를 참조하여 삭제해야 합니다.
    # 원본 4x2 그리드의 8번째 축을 참조하여 삭제 (axes.flatten()[7])
    # 단, axes는 이미 슬라이싱 되었으므로, 원본 axes를 참조해야 합니다.
    
    # **안전한 삭제 방법:**
    # 4x2 배열의 마지막 빈 칸 (4행 2열)을 직접 삭제합니다.
    axes_all = plt.gcf().get_axes()
    if len(axes_all) > 7:
        plt.gcf().delaxes(axes_all[-1])
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

# --- 7. 메인 실행 ---
if __name__ == '__main__':
    data_frames = load_and_normalize_data(max_duration=4.0)
    
    if data_frames:
        print("\n--- 시각화 시작 ---")
        
        plot_h2_activation(data_frames)
        plot_3d_ee_trajectories(data_frames)
        plot_3d_link4_trajectories(data_frames)
        plot_joint_trajectories(data_frames)
    else:
        print("\n시각화를 위해 로그 파일 3개 중 최소한 하나가 올바른 경로에 있는지 확인해주세요.")