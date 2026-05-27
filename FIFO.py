import os
import sys
import time
import math
import traci
import json

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("환경변수 'SUMO_HOME'을 선언해주세요.")

# =========================================================
# 시스템 제어 상수 설정
# =========================================================
GLOBAL_MAX_SPEED = 16.0
NORMAL_ACCEL = 2.0
MAINTAIN = 0.0
NORMAL_DECEL = -2.0
EMERGENCY_DECEL = -5.0
STEP_LENGTH = 0.01

TARGET_LOG_N = 24  # 🎯 추적할 JSON 차량 번호 (예: 24번 차량. 음수면 끔)

# =========================================================
# 1. 경로 및 충돌 지점(CP) 절대 거리 정의
# =========================================================
ROUTE_CP_DISTANCES = {
    "route_N_to_S_0": {"cp_1": 94.75, "cp_5": 98.25, "cp_12": 101.75, "cp_17": 105.25, "cp_23": 111.0},
    "route_N_to_S_1": {"cp_2": 94.75, "cp_6": 98.25, "cp_10": 100.0, "cp_13": 101.75, "cp_18": 105.25},
    "route_S_to_N_0": {"cp_20": 94.75, "cp_16": 98.25, "cp_9": 101.75, "cp_4": 105.25, "cp_22": 111.0},
    "route_S_to_N_1": {"cp_19": 94.75, "cp_15": 98.25, "cp_11": 100.0, "cp_8": 101.75, "cp_3": 105.25},
    "route_E_to_W_0": {"cp_4": 94.75, "cp_3": 98.25, "cp_2": 101.75, "cp_1": 105.25, "cp_21": 111.0},
    "route_E_to_W_1": {"cp_9": 94.75, "cp_8": 98.25, "cp_7": 100.0, "cp_6": 101.75, "cp_5": 105.25},
    "route_W_to_E_0": {"cp_17": 94.75, "cp_18": 98.25, "cp_19": 101.75, "cp_20": 105.25, "cp_24": 111.0},
    "route_W_to_E_1": {"cp_12": 94.75, "cp_13": 98.25, "cp_14": 100.0, "cp_15": 101.75, "cp_16": 105.25},
    "route_N_to_E_1": {"cp_2": 94.75, "cp_7": 98.66, "cp_11": 101.13, "cp_16": 105.04},
    "route_S_to_W_1": {"cp_19": 94.75, "cp_14": 98.66, "cp_10": 101.13, "cp_5": 105.04},
    "route_E_to_S_1": {"cp_9": 94.75, "cp_11": 98.66, "cp_14": 101.13, "cp_18": 105.04},
    "route_W_to_N_1": {"cp_12": 94.75, "cp_10": 98.66, "cp_7": 101.13, "cp_3": 105.04},
    "route_N_to_W_0": {"cp_21": 98.50},
    "route_E_to_N_0": {"cp_22": 98.50},
    "route_W_to_S_0": {"cp_23": 98.50},
    "route_S_to_E_0": {"cp_24": 98.50}
}

RAW_CPS = {
    "cp_1": (-5.25, 5.25), "cp_2": (-1.75, 5.25), "cp_3": (1.75, 5.25), "cp_4": (5.25, 5.25),
    "cp_5": (-5.25, 1.75), "cp_6": (-1.75, 1.75), "cp_7": (0.0, 1.75), "cp_8": (1.75, 1.75), "cp_9": (5.25, 1.75),
    "cp_10": (-1.75, 0.0), "cp_11": (1.75, 0.0),
    "cp_12": (-5.25, -1.75), "cp_13": (-1.75, -1.75), "cp_14": (0.0, -1.75), "cp_15": (1.75, -1.75),
    "cp_16": (5.25, -1.75),
    "cp_17": (-5.25, -5.25), "cp_18": (-1.75, -5.25), "cp_19": (1.75, -5.25), "cp_20": (5.25, -5.25),
    "cp_21": (-11, 5.25), "cp_22": (5.25, 11), "cp_23": (-5.25, -11), "cp_24": (11, -5.25)
}

ROUTE_CONFLICT_MAP = {
    ("n_in", "s_out", 0): ["cp_1", "cp_5", "cp_12", "cp_17", "cp_23"],
    ("n_in", "s_out", 1): ["cp_2", "cp_6", "cp_10", "cp_13", "cp_18"],
    ("s_in", "n_out", 0): ["cp_20", "cp_16", "cp_9", "cp_4", "cp_22"],
    ("s_in", "n_out", 1): ["cp_19", "cp_15", "cp_11", "cp_8", "cp_3"],
    ("e_in", "w_out", 0): ["cp_4", "cp_3", "cp_2", "cp_1", "cp_21"],
    ("e_in", "w_out", 1): ["cp_9", "cp_8", "cp_7", "cp_6", "cp_5"],
    ("w_in", "e_out", 0): ["cp_17", "cp_18", "cp_19", "cp_20", "cp_24"],
    ("w_in", "e_out", 1): ["cp_12", "cp_13", "cp_14", "cp_15", "cp_16"],
    ("n_in", "e_out", 1): ["cp_2", "cp_7", "cp_11", "cp_16"],
    ("s_in", "w_out", 1): ["cp_19", "cp_14", "cp_10", "cp_5"],
    ("e_in", "s_out", 1): ["cp_9", "cp_11", "cp_14", "cp_18"],
    ("w_in", "n_out", 1): ["cp_12", "cp_10", "cp_7", "cp_3"],
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

def get_inferred_lane(from_edge, to_edge, current_lane, pos):
    if (from_edge, to_edge) in [("n_in", "e_out"), ("e_in", "s_out"), ("s_in", "w_out"), ("w_in", "n_out")]: return 1
    if (from_edge, to_edge) in [("n_in", "w_out"), ("e_in", "n_out"), ("s_in", "e_out"), ("w_in", "s_out")]: return 0
    return current_lane


def get_route_key(from_edge, to_edge, lane):
    edge_map = {"n_in": "N", "s_in": "S", "e_in": "E", "w_in": "W",
                "n_out": "N", "s_out": "S", "e_out": "E", "w_out": "W"}
    if from_edge in edge_map and to_edge in edge_map:
        return f"route_{edge_map[from_edge]}_to_{edge_map[to_edge]}_{lane}"
    return None


class ConflictPoint:
    def __init__(self, cp_id):
        self.id = cp_id
        self.reservations = []

    def clear(self): self.reservations = []

    def add(self, car_id, t_in, t_out): self.reservations.append((car_id, t_in, t_out))


conflict_points = {cp_id: ConflictPoint(cp_id) for cp_id in RAW_CPS.keys()}
vehicle_depart_times = {}


def init_conflict_points():
    global conflict_points, vehicle_depart_times
    conflict_points = {cp_id: ConflictPoint(cp_id) for cp_id in RAW_CPS.keys()}
    vehicle_depart_times = {}


def generate_gui_settings():
    xml_content = """<viewsettings>
    <scheme name="real world"/>
    <vehicles vehicleName_show="1" vehicleName_size="60.00" vehicleName_color="blue"/>
</viewsettings>"""
    with open("gui-settings.xml", "w", encoding="utf-8") as f: f.write(xml_content)


def spawn_vehicle(v_id, r_id, lane, speed):
    traci.vehicle.add(v_id, r_id, depart="now", departLane=str(lane), departSpeed=speed)
    traci.vehicle.setMaxSpeed(v_id, GLOBAL_MAX_SPEED)
    traci.vehicle.setShapeClass(v_id, "passenger")


# =========================================================
# 3. 통합 예측 제어 (MPC 기반 FIFO)
# =========================================================
def calculate_eta(dist, v, a):
    if dist <= 0: return 0.0
    if abs(a) < 0.001: return (dist / v) if v > 0.001 else float('inf')
    disc = v ** 2 + 2 * a * dist
    if disc < 0: return float('inf')
    t = (-v + math.sqrt(disc)) / a
    return t if t >= 0 else float('inf')


def check_rear_end(car_speed, leader_speed, dist):
    SAFE_GAP = 2.5
    if dist <= SAFE_GAP: return True
    my_stop_dist = (car_speed ** 2) / (2 * abs(NORMAL_DECEL))
    leader_stop_dist = (leader_speed ** 2) / (2 * abs(NORMAL_DECEL))
    return my_stop_dist > (dist + leader_stop_dist - SAFE_GAP - 1.0)


def apply_acceleration(veh_id, accel_cmd):
    curr_v = traci.vehicle.getSpeed(veh_id)
    new_v = max(0.0, curr_v + (accel_cmd * STEP_LENGTH))
    traci.vehicle.setSpeed(veh_id, min(new_v, GLOBAL_MAX_SPEED))


def global_scheduling_fifo():
    veh_ids = traci.vehicle.getIDList()
    if not veh_ids: return
    curr_t = traci.simulation.getTime()

    for veh in veh_ids:
        if veh not in vehicle_depart_times:
            vehicle_depart_times[veh] = curr_t
            traci.vehicle.setSpeedMode(veh, 0)
            traci.vehicle.setLaneChangeMode(veh, 0)
            traci.vehicle.setMaxSpeed(veh, GLOBAL_MAX_SPEED)

    sorted_vehs = sorted(veh_ids, key=lambda x: vehicle_depart_times[x])
    for cp in conflict_points.values(): cp.clear()

    for veh in sorted_vehs:
        pos = traci.vehicle.getPosition(veh)
        speed = max(traci.vehicle.getSpeed(veh), 0.1)
        length = traci.vehicle.getLength(veh)

        try: veh_json_idx = int(veh.split('_')[-1])
        except: veh_json_idx = -1

        try:
            route = traci.vehicle.getRoute(veh)
            f, t = route[0], route[-1]
            lane = get_inferred_lane(f, t, traci.vehicle.getLaneIndex(veh), pos)
            target_cps = ROUTE_CONFLICT_MAP.get((f, t, lane), [])
            route_key = get_route_key(f, t, lane)
        except:
            continue

        # 출구 도로 진입 시
        current_road = traci.vehicle.getRoadID(veh)
        if current_road.endswith("_out"):
            max_limit_a = NORMAL_ACCEL
            leader = traci.vehicle.getLeader(veh, 50.0)
            leader_reason = ""
            if leader:
                l_id, l_dist = leader
                l_speed = max(traci.vehicle.getSpeed(l_id), 0.0)
                if check_rear_end(speed, l_speed, l_dist):
                    max_limit_a = NORMAL_DECEL
                    leader_reason = f"앞차({l_id}) 추돌 위험"
                    if l_dist < 5.0 and speed > l_speed:
                        max_limit_a = EMERGENCY_DECEL
            apply_acceleration(veh, max_limit_a)
            
            if veh_json_idx == TARGET_LOG_N:
                action = "ACCEL" if max_limit_a == NORMAL_ACCEL else ("DECEL" if max_limit_a == NORMAL_DECEL else "EMERGENCY")
                reason = leader_reason if leader_reason else "교차로 탈출 완료 (자유 주행)"
                Nth_car_state_log_print(TARGET_LOG_N, veh, curr_t, speed, max_limit_a, action, reason)
            continue

        # 1. 후방 추돌 방지
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

        dist_driven = traci.vehicle.getDistance(veh)

        # 2. 교차로 완전 통과 차량 검사
        passed_all = True
        if route_key and route_key in ROUTE_CP_DISTANCES:
            for cp_id in target_cps:
                cp_abs_dist = ROUTE_CP_DISTANCES[route_key].get(cp_id, 9999.0)
                dist_to_cp = cp_abs_dist - dist_driven
                if dist_to_cp >= - (length * 2):
                    passed_all = False
                    break
        else:
            passed_all = False

        if passed_all:
            apply_acceleration(veh, max_limit_a)
            if veh_json_idx == TARGET_LOG_N:
                action = "ACCEL" if max_limit_a == NORMAL_ACCEL else ("DECEL" if max_limit_a == NORMAL_DECEL else "EMERGENCY")
                reason = leader_reason if leader_reason else "교차로 통과 완료 (자유 가속)"
                Nth_car_state_log_print(TARGET_LOG_N, veh, curr_t, speed, max_limit_a, action, reason)
            continue

        # 3. 교차로 내부 차량 - 통합 예측 평가
        best_a = EMERGENCY_DECEL
        cp_times = []
        cp_reason = ""

        if route_key and route_key in ROUTE_CP_DISTANCES:
            for cp_id in target_cps:
                cp_abs_dist = ROUTE_CP_DISTANCES[route_key].get(cp_id, 9999.0)
                dist_to_cp = cp_abs_dist - dist_driven

                if dist_to_cp >= - (length):
                    t_in_d = calculate_eta(max(dist_to_cp - length, 0.1), speed, 0.0)
                    t_out_d = calculate_eta(max(dist_to_cp + (length), 0.1), speed, 0.0)

                    if t_in_d != float('inf') and t_out_d != float('inf'):
                        cp_times.append((cp_id, curr_t + t_in_d, curr_t + t_out_d))

        best_res = list(cp_times)

        candidates = [NORMAL_ACCEL, MAINTAIN, NORMAL_DECEL, EMERGENCY_DECEL]
        candidates = [c for c in candidates if c <= max_limit_a]

        for a_cand in candidates:
            success = True
            # 가속도 후보별 안전 마진 동적 할당
            margin = 0.25 if a_cand == NORMAL_ACCEL else 0.0
            
            for cp_id, t_in, t_out in cp_times:
                for r_veh, r_in, r_out in conflict_points[cp_id].reservations:
                    # 마진(margin)을 반영한 시간표 중첩 판별
                    if t_in < r_out + margin and r_in < t_out + margin:
                        success = False
                        cp_reason = f"선순위 {r_veh}와 중첩 (마진 {margin}s 미달)"
                        break
                if not success: break

            if success:
                best_a = a_cand
                best_res = list(cp_times)
                break

        # 4. 명령 확정 및 예약 동기화
        for cp_id, tin, tout in best_res:
            conflict_points[cp_id].add(veh, tin, tout)
        apply_acceleration(veh, best_a)
        
        if veh_json_idx == TARGET_LOG_N:
            action = "ACCEL" if best_a == NORMAL_ACCEL else ("MAINTAIN" if best_a == MAINTAIN else ("DECEL" if best_a == NORMAL_DECEL else "EMERGENCY"))
            final_reason = leader_reason if leader_reason else (cp_reason if cp_reason else "안전 경로 확보")
            Nth_car_state_log_print(TARGET_LOG_N, veh, curr_t, speed, best_a, action, final_reason)

# =========================================================
# 4. 메인 실행 블록 (단일 실행 모드)
# =========================================================
if __name__ == "__main__":
    scenario_file = 'scenario_40.json'
    if not os.path.exists(scenario_file):
        sys.exit(f"'{scenario_file}' 파일이 없습니다. 시나리오 파일이 필요합니다.")

    generate_gui_settings()
    init_conflict_points()

    with open(scenario_file, 'r') as f:
        scenario = json.load(f)

    traci.start([
        "sumo-gui", "-n", "main.net.xml",
        "--step-length", "0.01",
        "--collision.action", "none",
        "--gui-settings-file", "gui-settings.xml"
    ])

    for d in scenario:
        try:
            traci.route.add(d["route_id"], [d["origin_edge"], d["to_edge"]])
        except:
            pass
        spawn_vehicle(d["veh_id"], d["route_id"], d["lane"], d["speed"])

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        global_scheduling_fifo()

    traci.close()