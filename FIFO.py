import os
import sys
import time
import math
import traci
import threading
import queue
import random
import json

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("환경변수 'SUMO_HOME'을 선언해주세요.")

# =========================================================
# ⚙️ 시스템 제어 상수 및 로깅 설정
# =========================================================
GLOBAL_MAX_SPEED = 16.0  
NORMAL_ACCEL = 2.0      
MAINTAIN = 0.0          
NORMAL_DECEL = -2.0     
EMERGENCY_DECEL = -5.0  
STEP_LENGTH = 0.01      

TARGET_LOG_N = 16  # 추적할 JSON 차량 번호 (예: 20번 차량. 0~39 사이 입력. 음수면 끔)

# =========================================================
# 1. 경로(PATHS) 및 충돌 지점(CP) 정의
# =========================================================
PATHS = {
    ("n_in", "s_out"): {0: [(-5.25, 11.0), (-5.25, -11.0)], 1: [(-1.75, 11.0), (-1.75, -11.0)]},
    ("s_in", "n_out"): {0: [(5.25, -11.0), (5.25, 11.0)], 1: [(1.75, -11.0), (1.75, 11.0)]},
    ("e_in", "w_out"): {0: [(11.0, 5.25), (-11.0, 5.25)], 1: [(11.0, 1.75), (-11.0, 1.75)]},
    ("w_in", "e_out"): {0: [(-11.0, -5.25), (11.0, -5.25)], 1: [(-11.0, -1.75), (11.0, -1.75)]},
    ("n_in", "e_out"): {1: [(-1.75, 11.00), (-1.75, 5.25), (0.00, 1.75), (1.75, 0.00), (5.25, -1.75), (11.00, -1.75)]},
    ("e_in", "s_out"): {1: [(11.00, 1.75), (5.25, 1.75), (1.75, 0.00), (0.00, -1.75), (-1.75, -5.25), (-1.75, -11.00)]},
    ("s_in", "w_out"): {1: [(1.75, -11.00), (1.75, -5.25), (0.00, -1.75), (-1.75, 0.00), (-5.25, 1.75), (-11.00, 1.75)]},
    ("w_in", "n_out"): {1: [(-11.00, -1.75), (-5.25, -1.75), (-1.75, 0.00), (0.00, 1.75), (1.75, 5.25), (1.75, 11.00)]},
    ("n_in", "w_out"): {0: [(-5.25, 11.00), (-5.61, 8.48), (-6.69, 6.69), (-8.48, 5.61), (-11.00, 5.25)]},
    ("e_in", "n_out"): {0: [(11.00, 5.25), (8.48, 5.61), (6.69, 6.69), (5.61, 8.48), (5.25, 11.00)]},
    ("s_in", "e_out"): {0: [(5.25, -11.00), (5.61, -8.48), (6.69, -6.69), (8.48, -5.61), (11.00, -5.25)]},
    ("w_in", "s_out"): {0: [(-11.00, -5.25), (-8.48, -5.61), (-6.69, -6.69), (-5.61, -8.48), (-5.25, -11.00)]}
}

RAW_CPS = {
    "cp_1": (-5.25, 5.25), "cp_2": (-1.75, 5.25), "cp_3": (1.75, 5.25), "cp_4": (5.25, 5.25),
    "cp_5": (-5.25, 1.75), "cp_6": (-1.75, 1.75), "cp_7": (0.0, 1.75), "cp_8": (1.75, 1.75), "cp_9": (5.25, 1.75),
    "cp_10": (-1.75, 0.0), "cp_11": (1.75, 0.0),
    "cp_12": (-5.25, -1.75), "cp_13": (-1.75, -1.75), "cp_14": (0.0, -1.75), "cp_15": (1.75, -1.75), "cp_16": (5.25, -1.75),
    "cp_17": (-5.25, -5.25), "cp_18": (-1.75, -5.25), "cp_19": (1.75, -5.25), "cp_20": (5.25, -5.25), 
    "cp_21": (-11, 5.25), "cp_22": (5.25, 11), "cp_23": (-5.25, -11), "cp_24": (11, -5.25)
}

ROUTE_CONFLICT_MAP = {
    #직진
    ("n_in", "s_out", 0): ["cp_1", "cp_5", "cp_12", "cp_17", "cp_23"],
    ("n_in", "s_out", 1): ["cp_2", "cp_6", "cp_10", "cp_13", "cp_18"],
    ("s_in", "n_out", 0): ["cp_20", "cp_16", "cp_9", "cp_4", "cp_22"],
    ("s_in", "n_out", 1): ["cp_19", "cp_15", "cp_11", "cp_8", "cp_3"],
    ("e_in", "w_out", 0): ["cp_4", "cp_3", "cp_2", "cp_1", "cp_21"],
    ("e_in", "w_out", 1): ["cp_9", "cp_8", "cp_7", "cp_6", "cp_5"],
    ("w_in", "e_out", 0): ["cp_17", "cp_18", "cp_19", "cp_20", "cp_24"],
    ("w_in", "e_out", 1): ["cp_12", "cp_13", "cp_14", "cp_15", "cp_16"],
    #좌회전
    ("n_in", "e_out", 1): ["cp_2", "cp_7", "cp_11", "cp_16"],
    ("s_in", "w_out", 1): ["cp_19", "cp_14", "cp_10", "cp_5"],
    ("e_in", "s_out", 1): ["cp_9", "cp_11", "cp_14", "cp_18"],
    ("w_in", "n_out", 1): ["cp_12", "cp_10", "cp_7", "cp_3"],
    #우회전
    ("n_in", "w_out", 0): ["cp_21"], ("s_in", "e_out", 0): ["cp_24"],
    ("e_in", "n_out", 0): ["cp_22"], ("w_in", "s_out", 0): ["cp_23"]
}

# =========================================================
# 2. 유틸리티 함수 및 클래스
# =========================================================
def Nth_car_state_log_print(target_n, veh_id, current_time, speed, accel_cmd, action_str, reason):
    if target_n < 0: return
    is_max = (speed >= GLOBAL_MAX_SPEED - 0.05)
    accel_display = "최고속도" if is_max and action_str == "ACCEL" else f"{accel_cmd:+.1f}m/s²"
    print(f"🎯 [JSON NO.{target_n} 차: {veh_id}] Time: {current_time:.2f}s | 현재속도: {speed:>4.1f}m/s | 가속명령: {accel_display:>7} | 상태: {action_str:<9} | 사유: {reason}")

def get_euclidean(pos1, pos2): return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5
def is_point_on_segment(pt, A, B, tol=0.1): return abs(get_euclidean(A, B) - (get_euclidean(A, pt) + get_euclidean(B, pt))) < tol

def get_exact_dist(current_pos, cp, target_path):
    segment_idx = -1
    for j in range(len(target_path) - 1):
        if is_point_on_segment(cp, target_path[j], target_path[j+1]):
            segment_idx = j; break
    if segment_idx == -1: return 9999.0 
    dist = 0.0
    if (current_pos[0]**2 + current_pos[1]**2)**0.5 > 11.5:
        dist += get_euclidean(current_pos, target_path[0])
        for j in range(segment_idx): dist += get_euclidean(target_path[j], target_path[j+1])
        dist += get_euclidean(target_path[segment_idx], cp)
    else:
        min_d = 9999.0; veh_seg_idx = 0
        for j in range(len(target_path)):
            d = get_euclidean(current_pos, target_path[j])
            if d < min_d: min_d = d; veh_seg_idx = j
        if veh_seg_idx > segment_idx: return -1.0 
        if veh_seg_idx == segment_idx:
            dist_to_cp = get_euclidean(current_pos, cp)
            if get_euclidean(target_path[segment_idx], current_pos) > get_euclidean(target_path[segment_idx], cp): return -1.0
            dist = dist_to_cp
        else:
            dist += get_euclidean(current_pos, target_path[veh_seg_idx+1])
            for j in range(veh_seg_idx + 1, segment_idx): dist += get_euclidean(target_path[j], target_path[j+1])
            dist += get_euclidean(target_path[segment_idx], cp)
    return dist

def get_inferred_lane(from_edge, to_edge, current_lane, pos):
    if (from_edge, to_edge) in [("n_in", "e_out"), ("e_in", "s_out"), ("s_in", "w_out"), ("w_in", "n_out")]: return 1
    if (from_edge, to_edge) in [("n_in", "w_out"), ("e_in", "n_out"), ("s_in", "e_out"), ("w_in", "s_out")]: return 0
    return current_lane

class ConflictPoint:
    def __init__(self, cp_id, x, y):
        self.id = cp_id; self.x = x; self.y = y
        self.reservations = []
    def clear(self): self.reservations = []
    def add(self, car_id, t_in, t_out): self.reservations.append((car_id, t_in, t_out))

conflict_points = {cp_id: ConflictPoint(cp_id, x, y) for cp_id, (x, y) in RAW_CPS.items()}
vehicle_depart_times = {}

def init_conflict_points():
    global conflict_points, vehicle_depart_times
    conflict_points = {cp_id: ConflictPoint(cp_id, x, y) for cp_id, (x, y) in RAW_CPS.items()}
    vehicle_depart_times = {}

def generate_gui_settings():
    xml_content = """<viewsettings>
    <scheme name="real world"/>
    <vehicles vehicleName_show="1" vehicleName_size="60.00" vehicleName_color="blue"/>
</viewsettings>"""
    with open("gui-settings.xml", "w", encoding="utf-8") as f: f.write(xml_content)

# =========================================================
# 3. 통합 예측 제어 (MPC 기반 FIFO)
# =========================================================
def calculate_eta(dist, v, a):
    if dist <= 0: return 0.0
    if abs(a) < 0.001: return (dist / v) if v > 0.001 else float('inf')
    disc = v**2 + 2 * a * dist
    if disc < 0: return float('inf')
    t = (-v + math.sqrt(disc)) / a
    return t if t >= 0 else float('inf')

def check_rear_end(car_speed, leader_speed, dist):
    SAFE_GAP = 2.5
    if dist <= SAFE_GAP: return True
    my_stop_dist = (car_speed**2) / (2 * abs(NORMAL_DECEL))
    leader_stop_dist = (leader_speed**2) / (2 * abs(NORMAL_DECEL))
    return my_stop_dist > (dist + leader_stop_dist - SAFE_GAP - 1.0)

def apply_acceleration(veh_id, accel_cmd):
    curr_v = traci.vehicle.getSpeed(veh_id)
    new_v = max(0.0, curr_v + (accel_cmd * STEP_LENGTH))
    traci.vehicle.setSpeed(veh_id, min(new_v, GLOBAL_MAX_SPEED))

def global_scheduling_fifo(mode="normal"):
    veh_ids = traci.vehicle.getIDList()
    if not veh_ids: return
    curr_t = traci.simulation.getTime()

    for veh in veh_ids:
        if veh not in vehicle_depart_times:
            vehicle_depart_times[veh] = curr_t
            traci.vehicle.setSpeedMode(veh, 0); traci.vehicle.setLaneChangeMode(veh, 0)
            traci.vehicle.setMaxSpeed(veh, GLOBAL_MAX_SPEED)

    sorted_vehs = sorted(veh_ids, key=lambda x: vehicle_depart_times[x])
    for cp in conflict_points.values(): cp.clear()

    for veh in sorted_vehs:
        pos = traci.vehicle.getPosition(veh); speed = max(traci.vehicle.getSpeed(veh), 0.1)
        length = traci.vehicle.getLength(veh)
        
        # 🎯 JSON 번호 추출 (예: 'veh_e_in_Straight_20' -> 20)
        try: veh_json_idx = int(veh.split('_')[-1])
        except: veh_json_idx = -1
        
        try:
            route = traci.vehicle.getRoute(veh); f, t = route[0], route[-1]
            lane = get_inferred_lane(f, t, traci.vehicle.getLaneIndex(veh), pos)
            target_cps = ROUTE_CONFLICT_MAP.get((f, t, lane), [])
            path = PATHS[(f, t)][lane]
        except: continue

        # [수정] 차량이 이미 출구 도로(*_out) 위에 있다면 스케줄링 대상에서 배제.
        #   - 충돌점 검사 / 예약 생성 모두 건너뛰고, 후방 추돌만 점검한 뒤 자유 가속.
        #   - 출구로 나간 차량이 자기 경로 끝 CP를 매 스텝 다시 예약하는 그림자 예약 문제 차단.
        current_road = traci.vehicle.getRoadID(veh)
        if current_road.endswith("_out"):
            max_limit_a = NORMAL_ACCEL
            leader_reason = ""
            leader = traci.vehicle.getLeader(veh, 50.0)
            if leader:
                l_id, l_dist = leader
                l_speed = max(traci.vehicle.getSpeed(l_id), 0.0)
                if check_rear_end(speed, l_speed, l_dist):
                    max_limit_a = NORMAL_DECEL
                    leader_reason = f"앞차({l_id}) 추돌 위험"
                    if l_dist < 5.0 and speed > l_speed:
                        max_limit_a = EMERGENCY_DECEL
            best_a = max_limit_a
            action = "ACCEL" if best_a == NORMAL_ACCEL else ("DECEL" if best_a == NORMAL_DECEL else "EMERGENCY")
            final_reason = leader_reason if leader_reason else "출구 도로 진입 (스케줄링 제외)"
            apply_acceleration(veh, best_a)
            if veh_json_idx == TARGET_LOG_N:
                Nth_car_state_log_print(TARGET_LOG_N, veh, curr_t, speed, best_a, action, final_reason)
            continue

        # 1. 후방 추돌 방지 (모든 차량 공통)
        max_limit_a = NORMAL_ACCEL
        leader_reason = ""
        leader = traci.vehicle.getLeader(veh, 50.0)
        if leader:
            l_id, l_dist = leader
            l_speed = max(traci.vehicle.getSpeed(l_id), 0.0)
            if check_rear_end(speed, l_speed, l_dist):
                max_limit_a = NORMAL_DECEL
                leader_reason = f"앞차({l_id}) 추돌 위험"
                if l_dist < 5.0 and speed > l_speed:
                    max_limit_a = EMERGENCY_DECEL

        # 2. 교차로 완전 통과 차량은 교차로 검사 생략하고 가감속만 적용
        passed_all = True
        for cp_id in target_cps:
            if get_exact_dist(pos, (RAW_CPS[cp_id][0], RAW_CPS[cp_id][1]), path) >= 0:
                passed_all = False
                break

        if passed_all:
            best_a = max_limit_a
            action = "ACCEL" if best_a == NORMAL_ACCEL else ("DECEL" if best_a == NORMAL_DECEL else "EMERGENCY")
            final_reason = leader_reason if leader_reason else "교차로 통과 완료 (자유 가속)"
            apply_acceleration(veh, best_a)
            if veh_json_idx == TARGET_LOG_N:
                Nth_car_state_log_print(TARGET_LOG_N, veh, curr_t, speed, best_a, action, final_reason)
            continue

        # 3. 교차로 내부 차량 - 통합 예측 평가4
        best_a = EMERGENCY_DECEL
        best_res = []
        cp_reason = ""

        cp_times = []  
        for cp_id in target_cps:
            dist = get_exact_dist(pos, (RAW_CPS[cp_id][0], RAW_CPS[cp_id][1]), path)
            if dist >= 0:
                t_in_d = calculate_eta(max(dist - length/2 - 2.0, 0.1), speed, 0.0)
                t_out_d = calculate_eta(max(dist + length/2 + 2.0, 0.1), speed, 0.0)
                if t_in_d == float('inf') or t_out_d == float('inf'):
                    continue
                cp_times.append((cp_id, curr_t + t_in_d, curr_t + t_out_d))

        candidates = [NORMAL_ACCEL, MAINTAIN, NORMAL_DECEL, EMERGENCY_DECEL]
        candidates = [c for c in candidates if c <= max_limit_a]

        for a_cand in candidates:
            success = True
            for cp_id, t_in, t_out in cp_times:
                for r_veh, r_in, r_out in conflict_points[cp_id].reservations:
                    if max(t_in, r_in) < min(t_out, r_out):
                        success = False
                        cp_reason = f"선순위 {r_veh}와 예약 중첩"
                        break
                if not success:
                    break

            if success:
                best_a = a_cand
                best_res = list(cp_times)
                break

        # 4. 명령 확정 및 예약 동기화
        action = "ACCEL" if best_a == NORMAL_ACCEL else ("MAINTAIN" if best_a == MAINTAIN else ("DECEL" if best_a == NORMAL_DECEL else "EMERGENCY"))
        final_reason = leader_reason if leader_reason else (cp_reason if cp_reason else "안전 경로 확보")
        
        for cp_id, tin, tout in best_res: conflict_points[cp_id].add(veh, tin, tout)
        apply_acceleration(veh, best_a)

        if veh_json_idx == TARGET_LOG_N:
            Nth_car_state_log_print(TARGET_LOG_N, veh, curr_t, speed, best_a, action, final_reason)

# =========================================================
# 4. 실행 모드 (1~4)
# =========================================================
def spawn_vehicle(v_id, r_id, lane, speed):
    traci.vehicle.add(v_id, r_id, depart="now", departLane=str(lane), departSpeed=speed)
    traci.vehicle.setMaxSpeed(v_id, GLOBAL_MAX_SPEED)
    traci.vehicle.setShapeClass(v_id, "passenger") # 승용차 모양으로 고정

def inject_crashing_vehicles():
    traci.route.add("r_crash_N2S", ["n_in", "s_out"])
    traci.route.add("r_crash_E2W", ["e_in", "w_out"])
    spawn_vehicle("veh_Fast", "r_crash_E2W", "0", 10.0)
    spawn_vehicle("veh_Slow", "r_crash_N2S", "0", 8.0)

def run_mode_2():
    print("\n▶ [모드 2] 충돌 A/B 테스트")
    generate_gui_settings(); init_conflict_points()
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.01", "--collision.action", "none", "--gui-settings-file", "gui-settings.xml"])
    inject_crashing_vehicles()
    
    crash_time = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if traci.simulation.getCollidingVehiclesNumber() > 0:
            crash_time = round(traci.simulation.getTime(), 2)
            print(f"💥 {crash_time}s 충돌 발생!"); break
    traci.close(); time.sleep(1)
    
    init_conflict_points()
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.01", "--collision.action", "none", "--gui-settings-file", "gui-settings.xml"])
    inject_crashing_vehicles()
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep(); global_scheduling_fifo(mode="ep2")
    traci.close()

def run_mode_3():
    print("\n▶ [모드 3] 40대 랜덤 생성 및 시나리오 저장")
    generate_gui_settings(); init_conflict_points()
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.01", "--collision.action", "none", "--gui-settings-file", "gui-settings.xml"])
    origins = ["e_in", "w_in", "s_in", "n_in"]; scenario = []; cnt = 0
    
    for o in origins:
        for _ in range(10):
            spd = random.uniform(8.0, 16.0); turn = random.choice(["Straight", "Left", "Right"])
            if o=="e_in": d="w_out" if turn=="Straight" else ("s_out" if turn=="Left" else "n_out")
            elif o=="w_in": d="e_out" if turn=="Straight" else ("n_out" if turn=="Left" else "s_out")
            elif o=="s_in": d="n_out" if turn=="Straight" else ("w_out" if turn=="Left" else "e_out")
            else: d="s_out" if turn=="Straight" else ("e_out" if turn=="Left" else "w_out")
            
            lane = "1" if turn == "Left" else ("0" if turn == "Right" else str(random.choice([0, 1])))
            
            vid = f"veh_{o}_{turn}_{cnt}"; rid = f"r_{cnt}"
            try: traci.route.add(rid, [o, d])
            except: pass
            
            spawn_vehicle(vid, rid, lane, spd)
            scenario.append({"veh_id": vid, "route_id": rid, "origin_edge": o, "to_edge": d, "speed": spd, "lane": lane}) 
            cnt += 1
            
    with open('scenario_40.json', 'w') as f: json.dump(scenario, f, indent=4)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep(); global_scheduling_fifo()
    traci.close()

def run_mode_4():
    print("\n▶ [모드 4] 시나리오 로드")
    if not os.path.exists('scenario_40.json'): print("❌ 'scenario_40.json' 파일이 없습니다."); return
    generate_gui_settings(); init_conflict_points()
    with open('scenario_40.json', 'r') as f: scenario = json.load(f)
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.01", "--collision.action", "none", "--gui-settings-file", "gui-settings.xml"])
    for d in scenario:
        try: traci.route.add(d["route_id"], [d["origin_edge"], d["to_edge"]])
        except: pass
        spawn_vehicle(d["veh_id"], d["route_id"], d["lane"], d["speed"])
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep(); global_scheduling_fifo(mode="ep2")
    traci.close()

if __name__ == "__main__":
    choice = input("모드 선택 (2: 충돌테스트, 3: 생성/저장, 4: 로드): ")
    if choice == '2': run_mode_2()
    elif choice == '3': run_mode_3()
    elif choice == '4': run_mode_4()