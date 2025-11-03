import matplotlib.pyplot as plt
import numpy as np
import os
import math
from matplotlib.ticker import MultipleLocator

# 라디안을 Degree로 변환하는 함수
def rad_to_deg(rad):
    return rad * 180.0 / math.pi

# --- 헬퍼 함수: 데이터 로드 ---
def load_data(filename):
    """지정된 로그 파일에서 데이터를 로드하고 3열 구조에 맞게 반환합니다."""
    if not os.path.exists(filename):
        print(f"오류: 파일을 찾을 수 없습니다: {filename}")
        return None, None, None
        
    try:
        data = np.loadtxt(filename)
    except Exception as e:
        print(f"오류: {filename} 파일을 로드하는 데 실패했습니다. 파일 형식 확인 필요. {e}")
        return None, None, None
        
    if data.ndim < 2 or data.shape[1] < 3:
        print(f"경고: {filename}의 열 개수가 예상(3개)보다 적습니다 (현재: {data.shape[1] if data.ndim >= 2 else 0}).")
        return None, None, None

    time = data[:, 0]
    qd_j4 = data[:, 1]
    q_j4 = data[:, 2]
    
    return time, qd_j4, q_j4

# --- 1. Step Response 시각화 함수 (plt.show() 추가) ---
def plot_step_response(time, qd_j4, q_j4, title, filename_out):
    """Joint 4의 스텝 응답을 시각화합니다."""
    if time is None: return
    
    time_relative = time - time[0]
    qd_j4_deg = rad_to_deg(qd_j4)
    q_j4_deg = rad_to_deg(q_j4)
    
    plt.figure(figsize=(10, 6))
    plt.plot(time_relative, qd_j4_deg, 'r--', label='Desired $q_4$ (Deg)')
    plt.plot(time_relative, q_j4_deg, 'b-', label='Actual $q_4$ (Deg)')
    
    ax = plt.gca()
    ax.xaxis.set_major_locator(MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(MultipleLocator(0.5))
    plt.grid(True, which='both', linestyle='--')
    plt.xlim(left=0)
    
    plt.title(f'Joint 4 Step Response ({title})')
    plt.xlabel('Time (sec)')
    plt.ylabel('Joint Position (Degree)')
    plt.legend()
    plt.savefig(filename_out)
    
    # 🌟 플롯 표시
    plt.show() 
    
    plt.close()

# --- 2. Trajectory Tracking 시각화 함수 (plt.show() 추가) ---
def plot_trajectory_tracking(time, qd_j4, q_j4, title, filename_out):
    """Joint 4의 궤적 추종 결과를 시각화합니다."""
    if time is None: return
    
    time_relative = time - time[0]
    qd_j4_deg = rad_to_deg(qd_j4)
    q_j4_deg = rad_to_deg(q_j4)
    
    plt.figure(figsize=(10, 6))
    plt.plot(time_relative, qd_j4_deg, 'r--', label='Desired $q_4$ Trajectory (Deg)')
    plt.plot(time_relative, q_j4_deg, 'b-', label='Actual $q_4$ Position (Deg)')
    
    ax = plt.gca()
    ax.xaxis.set_major_locator(MultipleLocator(1.0))
    ax.xaxis.set_minor_locator(MultipleLocator(0.5))
    plt.grid(True, which='both', linestyle='--')
    plt.xlim(left=0)
    
    plt.title(f'Joint 4 Trajectory Tracking ({title})')
    plt.xlabel('Time (sec)')
    plt.ylabel('Joint Position (Degree)')
    plt.legend()
    plt.savefig(filename_out)
    
    # 🌟 플롯 표시
    plt.show()
    
    plt.close()

# =================================================================
#                         시각화 실행 부분
# =================================================================
print("Starting Visualization...")

# --- 문제 2: Simple PD Controller (Step Response) ---
print("Plotting Problem 2: Simple PD Step Response")
time_2, qd_2, q_2 = load_data("PD_Step.txt")
plot_step_response(time_2, qd_2, q_2, "Simple PD Controller", "P2_PD_Step_Response.png")

# --- 문제 3: PD with Gravity Compensation ---
print("\nPlotting Problem 3...")
time_3s, qd_3s, q_3s = load_data("PDwGravityComp_Step.txt")
plot_step_response(time_3s, qd_3s, q_3s, "PD with Gravity Compensation (Step)", "P3_PDwGC_Step_Response.png")

time_3t, qd_3t, q_3t = load_data("PDwGravityComp_Spline.txt")
plot_trajectory_tracking(time_3t, qd_3t, q_3t, "PD with Gravity Compensation (Tracking)", "P3_PDwGC_Trajectory_Tracking.png")

# --- 문제 4: PD with Dynamic Compensation ---
print("\nPlotting Problem 4...")
time_4s, qd_4s, q_4s = load_data("DynamicComp_Step.txt")
plot_step_response(time_4s, qd_4s, q_4s, "PD with Dynamic Compensation (Step)", "P4_PDwDC_Step_Response.png")

time_4t, qd_4t, q_4t = load_data("DynamicComp_Spline.txt")
plot_trajectory_tracking(time_4t, qd_4t, q_4t, "PD with Dynamic Compensation (Tracking)", "P4_PDwDC_Trajectory_Tracking.png")

print("\nVisualization Complete. Check the generated PNG files.")