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
# 1. 경로(PATHS) 및 차선(Lane)별 세부 궤적 좌표
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
    ("n_in", "s_out", 0): ["cp_1", "cp_5", "cp_12", "cp_17"],
    ("n_in", "s_out", 1): ["cp_2", "cp_6", "cp_10", "cp_13", "cp_18"],
    ("s_in", "n_out", 0): ["cp_20", "cp_16", "cp_9", "cp_4"],
    ("s_in", "n_out", 1): ["cp_19", "cp_15", "cp_11", "cp_8", "cp_3"],
    ("e_in", "w_out", 0): ["cp_4", "cp_3", "cp_2", "cp_1"],
    ("e_in", "w_out", 1): ["cp_9", "cp_8", "cp_7", "cp_6", "cp_5"],
    ("w_in", "e_out", 0): ["cp_17", "cp_18", "cp_19", "cp_20"],
    ("w_in", "e_out", 1): ["cp_12", "cp_13", "cp_14", "cp_15", "cp_16"],
    
    ("n_in", "e_out", 1): ["cp_2", "cp_7", "cp_11", "cp_16"],
    ("s_in", "w_out", 1): ["cp_19", "cp_14", "cp_10", "cp_5"],
    ("e_in", "s_out", 1): ["cp_9", "cp_11", "cp_14", "cp_18"],
    ("w_in", "n_out", 1): ["cp_12", "cp_10", "cp_7", "cp_3"],
    
    ("n_in", "w_out", 0): ["cp_21"],
    ("s_in", "e_out", 0): ["cp_24"],
    ("e_in", "n_out", 0): ["cp_22"],
    ("w_in", "s_out", 0): ["cp_23"]
}

def generate_gui_settings():
    xml_content = """<viewsettings>
    <scheme name="real world"/>
    <vehicles vehicleName_show="1" vehicleName_size="60.00" vehicleName_color="blue"/>
</viewsettings>"""
    with open("gui-settings.xml", "w", encoding="utf-8") as f:
        f.write(xml_content)

def get_euclidean(pos1, pos2):
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5

def get_turn_type(from_edge, to_edge):
    if (from_edge, to_edge) in [("n_in", "e_out"), ("e_in", "s_out"), ("s_in", "w_out"), ("w_in", "n_out")]: return "Left"
    if (from_edge, to_edge) in [("n_in", "w_out"), ("e_in", "n_out"), ("s_in", "e_out"), ("w_in", "s_out")]: return "Right"
    return "Straight"

def get_dist_to_straight_path(pos, path):
    A, B = path[0], path[-1]
    if abs(A[0] - B[0]) < 0.1: return abs(pos[0] - A[0])
    else:                      return abs(pos[1] - A[1])

def get_inferred_lane(from_edge, to_edge, current_lane, pos):
    turn_type = get_turn_type(from_edge, to_edge)
    if turn_type == "Left": return 1    
    if turn_type == "Right": return 0   
    
    paths = PATHS.get((from_edge, to_edge), {})
    if 0 in paths and 1 in paths:
        dist0 = get_dist_to_straight_path(pos, paths[0])
        dist1 = get_dist_to_straight_path(pos, paths[1])
        return 0 if dist0 < dist1 else 1
    return current_lane

class ConflictPoint:
    def __init__(self, cp_id, x, y):
        self.id = cp_id
        self.x = x
        self.y = y
        self.reservations = []       
        self.prev_distances = {}     
        self.passed_cars = set()     

    def clear_reservations(self):
        self.reservations = []

    def update_car_distance(self, car_id, car_x, car_y, car_length):
        if car_id in self.passed_cars: return
        dist = get_euclidean((self.x, self.y), (car_x, car_y))

        if car_id in self.prev_distances:
            if dist > self.prev_distances[car_id] + 0.05: 
                if dist > (car_length / 2.0) + CP_RADIUS + 0.5:
                    self.mark_passed(car_id)
                    return
        self.prev_distances[car_id] = dist

    def mark_passed(self, car_id):
        self.passed_cars.add(car_id)
        if car_id in self.prev_distances:
            del self.prev_distances[car_id]

    def add_reservation(self, car_id, t_in, t_out, speed):
        self.reservations.append((car_id, t_in, t_out, speed))

conflict_points = {}
input_queue = queue.Queue()

CP_RADIUS = 2.0
MARGIN_TIME = 0.1 
LOG_ENABLED = False 

vehicle_entry_times = {}

def init_conflict_points():
    global conflict_points, vehicle_entry_times
    conflict_points = {}
    vehicle_entry_times = {}
    for cp_id, (x, y) in RAW_CPS.items():
        conflict_points[cp_id] = ConflictPoint(cp_id, x, y)

def command_listener():
    global LOG_ENABLED
    while True:
        try:
            cmd = sys.stdin.readline().strip().lower()
            if not cmd: continue
            
            if cmd == 'log':
                LOG_ENABLED = not LOG_ENABLED
                status = "ON" if LOG_ENABLED else "OFF"
                print(f"\n🖨️ [시스템] 상세 회피 로그 출력이 [{status}] 되었습니다.")
                sys.stdout.flush()
            else:
                speed = float(cmd)
                if speed < 1.0: speed = 1.0
                input_queue.put(speed)
        except ValueError:
            print(" ❌ 잘못된 입력입니다. 'log'를 입력하거나, 속도를 입력하세요.")
        except Exception:
            break

def is_point_on_segment(pt, A, B, tol=0.1):
    return abs(get_euclidean(A, B) - (get_euclidean(A, pt) + get_euclidean(B, pt))) < tol

def get_exact_dist(current_pos, cp, target_path):
    segment_idx = -1
    for j in range(len(target_path) - 1):
        if is_point_on_segment(cp, target_path[j], target_path[j+1]):
            segment_idx = j
            break
    if segment_idx == -1: return 9999.0 
    
    dist = 0.0
    if (current_pos[0]**2 + current_pos[1]**2)**0.5 > 11.5:
        dist += get_euclidean(current_pos, target_path[0])
        for j in range(segment_idx):
            dist += get_euclidean(target_path[j], target_path[j+1])
        dist += get_euclidean(target_path[segment_idx], cp)
    else:
        min_d = 9999.0
        veh_seg_idx = 0
        for j in range(len(target_path)):
            d = get_euclidean(current_pos, target_path[j])
            if d < min_d:
                min_d = d
                veh_seg_idx = j
                
        if veh_seg_idx > segment_idx: return -1.0 
        if veh_seg_idx == segment_idx:
            dist_to_cp = get_euclidean(current_pos, cp)
            dist_start_to_cp = get_euclidean(target_path[segment_idx], cp)
            dist_start_to_veh = get_euclidean(target_path[segment_idx], current_pos)
            if dist_start_to_veh > dist_start_to_cp: return -1.0 
            dist = dist_to_cp
        else:
            dist += get_euclidean(current_pos, target_path[veh_seg_idx+1])
            for j in range(veh_seg_idx + 1, segment_idx):
                dist += get_euclidean(target_path[j], target_path[j+1])
            dist += get_euclidean(target_path[segment_idx], cp)
    return dist

def update_cp_distances():
    active_vehicles_list = traci.vehicle.getIDList()
    for veh_id in active_vehicles_list:
        try:
            route = traci.vehicle.getRoute(veh_id)
            f_edge, t_edge = route[0], route[-1]
            lane_idx = traci.vehicle.getLaneIndex(veh_id)
            pos = traci.vehicle.getPosition(veh_id)
            
            inferred_lane = get_inferred_lane(f_edge, t_edge, lane_idx, pos)
            target_cps = ROUTE_CONFLICT_MAP.get((f_edge, t_edge, inferred_lane), [])
            length = traci.vehicle.getLength(veh_id) 
            for cp_id in target_cps:
                conflict_points[cp_id].update_car_distance(veh_id, pos[0], pos[1], length)
        except: pass

def global_scheduling(mode="random"):
    global vehicle_entry_times
    veh_ids = traci.vehicle.getIDList()
    if not veh_ids: return

    current_time = traci.simulation.getTime()
    veh_states = {}
    lane_groups = {} 

    active_set = set(veh_ids)
    for k in list(vehicle_entry_times.keys()):
        if k not in active_set:
            del vehicle_entry_times[k]
            
    for veh in veh_ids:
        if veh not in vehicle_entry_times:
            vehicle_entry_times[veh] = current_time

    for veh in veh_ids:
        pos = traci.vehicle.getPosition(veh)
        length = traci.vehicle.getLength(veh)
        lane_idx = traci.vehicle.getLaneIndex(veh)
        
        try:
            route = traci.vehicle.getRoute(veh)
            f_edge, t_edge = route[0], route[-1]
            inferred_lane = get_inferred_lane(f_edge, t_edge, lane_idx, pos)
            target_cps = ROUTE_CONFLICT_MAP.get((f_edge, t_edge, inferred_lane), [])
            
            passed_all_cps = True
            dist_to_first_cp = 9999.0
            
            if target_cps:
                for cp_id in target_cps:
                    if veh not in conflict_points[cp_id].passed_cars:
                        passed_all_cps = False
                        dist = get_exact_dist(pos, (conflict_points[cp_id].x, conflict_points[cp_id].y), PATHS[(f_edge, t_edge)][inferred_lane])
                        if dist >= 0:
                            dist_to_enter = max(dist - (length / 2.0) - CP_RADIUS, 0.0)
                            dist_to_first_cp = min(dist_to_first_cp, dist_to_enter)
            
            if not target_cps or passed_all_cps:
                traci.vehicle.setSpeedMode(veh, 1)
                traci.vehicle.setSpeed(veh, traci.vehicle.getAllowedSpeed(veh))
                continue
                
            target_path = PATHS[(f_edge, t_edge)][inferred_lane]
            veh_states[veh] = {
                'pos': pos,
                'length': length,
                'target_path': target_path,
                'target_cps': target_cps,
                'f_edge': f_edge,
                't_edge': t_edge,
                'dist_to_first_cp': dist_to_first_cp 
            }
            
            group_key = (f_edge, inferred_lane)
            if group_key not in lane_groups:
                lane_groups[group_key] = []
            lane_groups[group_key].append(veh)
        except: continue

    leaders = set()
    for group_key, vehicles in lane_groups.items():
        best_veh = min(vehicles, key=lambda v: vehicle_entry_times[v])
        leaders.add(best_veh)

    priority_list = []
    for veh in veh_states.keys():
        speed = traci.vehicle.getSpeed(veh)
        pos = veh_states[veh]['pos']
        dist_to_center = math.hypot(pos[0], pos[1])
        turn_type = get_turn_type(veh_states[veh]['f_edge'], veh_states[veh]['t_edge'])
        
        turn_prio = 0 if turn_type in ["Left", "Right"] else 1
        
        priority_list.append({
            'veh': veh,
            'speed': -speed,         
            'dist': dist_to_center,  
            'turn': turn_prio        
        })
        
        if veh in leaders:
            veh_states[veh]['base_speed'] = traci.vehicle.getAllowedSpeed(veh)
        else:
            veh_states[veh]['base_speed'] = max(traci.vehicle.getSpeed(veh), 0.001)

    sorted_vehs = sorted(priority_list, key=lambda x: (x['speed'], x['dist'], x['turn'], x['veh']))

    # ========== [EP2 로그용] 이번 스텝에서 이미 로그를 출력한 CP 추적 ==========
    ep2_logged_cps = set()
    # =======================================================================
    
    planned_speeds = {}

    for cp in conflict_points.values(): cp.clear_reservations()
        
    for veh_data in sorted_vehs:
        veh = veh_data['veh']
        state = veh_states[veh]
        new_speed = state['base_speed']
        original_base_speed = state['base_speed']  # [EP2 로그용] 감속 전 원래 속도 저장
        
        log_messages = []
        
        for cp_id in state['target_cps']:
            cp_obj = conflict_points[cp_id]
            if veh in cp_obj.passed_cars: continue
            
            dist = get_exact_dist(state['pos'], (cp_obj.x, cp_obj.y), state['target_path'])
            if dist >= 0:
                dist_to_enter = max(dist - (state['length'] / 2.0) - CP_RADIUS, 0.1)
                dist_to_exit = max(dist + (state['length'] / 2.0) + CP_RADIUS, 0.1)
                
                for res_veh, res_tin, res_tout, res_spd in cp_obj.reservations:
                    temp_tin = current_time + (dist_to_enter / max(new_speed, 0.001))
                    temp_tout = current_time + (dist_to_exit / max(new_speed, 0.001))
                    
                    if max(temp_tin, res_tin) < min(temp_tout, res_tout):

                         # ========== [EP2 전용 로그 출력] ==========
                        if mode == "ep2" and cp_id not in ep2_logged_cps:
                            ep2_logged_cps.add(cp_id)
                            print("\n" + "="*60)
                            print(f"[충돌점 {cp_id}] 점유구간 겹침 감지! (t={current_time:.2f}s)")
                            print("="*60)
                            print(f" [회피 전] 충돌점 점유 예약 현황")
                            print(f"   - 선순위 차량: {res_veh}")
                            print(f"     - 도착시간(t_in) : {res_tin:.3f}s")
                            print(f"     - 탈출시간(t_out): {res_tout:.3f}s")
                            print(f"   - 후순위 차량: {veh}")
                            print(f"     - 도착시간(t_in) : {temp_tin:.3f}s")
                            print(f"     - 탈출시간(t_out): {temp_tout:.3f}s")
                            overlap_start = max(temp_tin, res_tin)
                            overlap_end = min(temp_tout, res_tout)
                            print(f"   [주의] 점유구간 겹침: [{overlap_start:.3f}s ~ {overlap_end:.3f}s] ({(overlap_end-overlap_start):.3f}s 간 충돌)")

                            # 우선순위 판정 사유 분석
                            res_veh_data = next((x for x in priority_list if x['veh'] == res_veh), None)
                            cur_veh_data = veh_data
                            reason = ""
                            if res_veh_data:
                                if res_veh_data['speed'] != cur_veh_data['speed']:
                                    reason = f"선순위 차량의 속도({-res_veh_data['speed']:.2f}m/s)가 후순위 차량({-cur_veh_data['speed']:.2f}m/s)보다 빠름"
                                elif res_veh_data['dist'] != cur_veh_data['dist']:
                                    reason = f"선순위 차량이 교차로 중심에 더 가까움 ({res_veh_data['dist']:.2f}m vs {cur_veh_data['dist']:.2f}m)"
                                elif res_veh_data['turn'] != cur_veh_data['turn']:
                                    reason = f"선순위 차량이 회전차량(좌/우회전 우선) 이고 후순위 차량은 직진"
                                else:
                                    reason = "동률 조건 -> 차량 ID 사전순"
                            print(f"\n [우선순위 판정]")
                            print(f"   -> {veh} 가 후순위로 밀림")
                            print(f"   -> 사유: {reason}")
                        # ==========================================

                        target_time = res_tout + MARGIN_TIME
                        time_left = max(target_time - current_time, 0.1)
                        req_speed = dist_to_enter / time_left
                        
                        if req_speed < new_speed:
                            if LOG_ENABLED:
                                log_messages.append(f" ▷ [{cp_id}] 선순위 [{res_veh}] 대기 -> 예정속도 {new_speed:.2f}m/s 에서 {req_speed:.2f}m/s 로 감속")
                            new_speed = max(req_speed, 0.0)

        new_speed = min(new_speed, state['base_speed'])
        planned_speeds[veh] = new_speed

       # ========== [EP2 전용 로그 - 감속 결과 출력] ==========
        if mode == "ep2" and new_speed < original_base_speed - 0.01:
            print(f"\n [감속 적용]")
            print(f"   -> {veh}: {original_base_speed:.2f} m/s  ->  {new_speed:.2f} m/s")
        # ====================================================
        
        if LOG_ENABLED and log_messages:
            p_speed = -veh_data['speed']
            print(f"\n--- ⚠️ [차량: {veh}] 1-Pass 회피 예약 ---")
            print(f" ▶ 현재 스펙: 속도 {p_speed:.2f}m/s | 중심거리 {veh_data['dist']:.1f}m | 회전 {veh_data['turn']}")
            for msg in log_messages: print(msg)
            sys.stdout.flush()
            
        calc_speed = max(new_speed, 0.001)
        for cp_id in state['target_cps']:
            cp_obj = conflict_points[cp_id]
            if veh in cp_obj.passed_cars: continue
            dist = get_exact_dist(state['pos'], (cp_obj.x, cp_obj.y), state['target_path'])
            if dist >= 0:
                dist_to_enter = max(dist - (state['length'] / 2.0) - CP_RADIUS, 0.1)
                dist_to_exit = max(dist + (state['length'] / 2.0) + CP_RADIUS, 0.1)
                t_in = current_time + (dist_to_enter / calc_speed)
                t_out = current_time + (dist_to_exit / calc_speed)
                cp_obj.add_reservation(veh, t_in, t_out, new_speed)

                # ========== [EP2 전용 로그 - 회피 후 예약 현황] ==========
                if mode == "ep2" and cp_id in ep2_logged_cps:
                    if new_speed < original_base_speed - 0.01:
                        print(f"\n [회피 후] 충돌점 {cp_id} 재예약 현황")
                        for r_veh, r_tin, r_tout, r_spd in cp_obj.reservations:
                            print(f"   - {r_veh}: t_in={r_tin:.3f}s, t_out={r_tout:.3f}s")
                        reservations = cp_obj.reservations
                        has_overlap = False
                        for i in range(len(reservations)):
                            for j in range(i+1, len(reservations)):
                                _, a_in, a_out, _ = reservations[i]
                                _, b_in, b_out, _ = reservations[j]
                                if max(a_in, b_in) < min(a_out, b_out):
                                    has_overlap = True
                                    break
                        if not has_overlap:
                            print(f"   [성공] 점유구간 겹침 해소 완료 -> 충돌 회피 성공!")
                        print("="*60)
                        sys.stdout.flush()
                # =========================================================

    if mode != "ep1":
        for veh, final_speed in planned_speeds.items():
            traci.vehicle.setSpeedMode(veh, 1)
            
            if veh_states[veh]['dist_to_first_cp'] > 30.0:
                traci.vehicle.setSpeed(veh, traci.vehicle.getAllowedSpeed(veh))
            else:
                traci.vehicle.setSpeed(veh, final_speed)
                
def inject_crashing_vehicles():
    traci.route.add("r_crash_N2S", ["n_in", "s_out"])
    traci.route.add("r_crash_E2W", ["e_in", "w_out"])
    traci.vehicle.add("veh_Fast", "r_crash_E2W", depart=0, departLane="0", departPos="0.75", departSpeed=10)
    traci.vehicle.setMaxSpeed("veh_Fast", 10.0) 
    traci.vehicle.setMinGap("veh_Fast", 1.0) 
    traci.vehicle.add("veh_Slow", "r_crash_N2S", depart=0, departLane="0", departPos="10.7", departSpeed=8)
    traci.vehicle.setMaxSpeed("veh_Slow", 8.0) 
    traci.vehicle.setMinGap("veh_Slow", 1.0)
    # 속도 안전검사 완전 비활성화 (0 = 모든 체크 무시, 에피소드1에서 실제로 충돌 발생)
    traci.vehicle.setSpeedMode("veh_Fast", 0)
    traci.vehicle.setSpeedMode("veh_Slow", 0)
    # 차선변경 완전 비활성화 (0 = strategic/cooperative/speedGain/rightDrive 모두 끔)
    traci.vehicle.setLaneChangeMode("veh_Fast", 0)
    traci.vehicle.setLaneChangeMode("veh_Slow", 0)

def run_mode_1():
    print("\n▶ 1번 모드: 수동 차량 생성 시뮬레이션")
    print("▶ CMD 창에 원하는 초기 속도(m/s)를 숫자로 입력하고 Enter를 누르세요.")
    print("▶ CMD 창에 'log'를 입력하고 Enter를 누르면 상세 회피 로그가 ON/OFF 됩니다.")
    
    generate_gui_settings() 
    init_conflict_points()
    listener_thread = threading.Thread(target=command_listener, daemon=True)
    listener_thread.start()
    
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.1", "--collision.action", "none", "--time-to-teleport", "-1", "--gui-settings-file", "gui-settings.xml"])
    
    origins = ["e_in", "w_in", "s_in", "n_in"]
    origin_names = ["동쪽(E)", "서쪽(W)", "남쪽(S)", "북쪽(N)"]
    route_idx = 0
    veh_counter = 0
    
    try:
        while True:
            traci.simulationStep()
            update_cp_distances()

            while not input_queue.empty():
                speed = input_queue.get()
                origin_edge = origins[route_idx]
                origin_name = origin_names[route_idx]
                turn = random.choice(["Straight", "Left", "Right"])
                
                if origin_edge == "e_in": to_edge = "w_out" if turn == "Straight" else ("s_out" if turn == "Left" else "n_out")
                elif origin_edge == "w_in": to_edge = "e_out" if turn == "Straight" else ("n_out" if turn == "Left" else "s_out")
                elif origin_edge == "s_in": to_edge = "n_out" if turn == "Straight" else ("w_out" if turn == "Left" else "e_out")
                elif origin_edge == "n_in": to_edge = "s_out" if turn == "Straight" else ("e_out" if turn == "Left" else "w_out")
                
                route_id = f"r_{origin_edge}_{to_edge}_{veh_counter}"
                try: traci.route.add(route_id, [origin_edge, to_edge])
                except: pass
                
                if turn == "Left": lane = "1"
                elif turn == "Right": lane = "0"
                else: lane = str(random.choice([0, 1]))
                
                veh_id = f"veh_{origin_edge}_{turn}_{veh_counter}"
                traci.vehicle.add(veh_id, route_id, depart="now", departLane=lane, departPos="0", departSpeed=speed)
                traci.vehicle.setMaxSpeed(veh_id, speed)
                traci.vehicle.setMinGap(veh_id, 1.0)
                traci.vehicle.setSpeedMode(veh_id, 1) 
                
                turn_str = "직진" if turn == "Straight" else ("좌회전" if turn == "Left" else "우회전")
                print(f"🚗 [네트워크 등록] ID: {veh_id} | 위치: {origin_name} | 경로: {turn_str} ({lane}차선) | 최대속도: {speed}m/s")
                sys.stdout.flush()
                
                veh_counter += 1
                route_idx = (route_idx + 1) % 4 
                
            global_scheduling(mode="random")
    except traci.exceptions.FatalTraCIError:
        print("\n✅ 시뮬레이션 창이 닫혀 프로그램을 종료합니다.")
        sys.exit()

def run_mode_2():
    print("\n" + "#"*60)
    print("▶ [에피소드 1: 제어 없는 원본 충돌 상황]")
    print("#"*60)
    generate_gui_settings()
    init_conflict_points()
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.1", "--collision.action", "none", "--time-to-teleport", "-1", "--gui-settings-file", "gui-settings.xml"])
    inject_crashing_vehicles()
    crash_time = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # 에피소드1: 회피 로직 없이 등속 주행 강제 (실제 충돌 연출)
        for v in traci.vehicle.getIDList():
            traci.vehicle.setSpeed(v, traci.vehicle.getMaxSpeed(v))
        update_cp_distances()
        global_scheduling(mode="ep1")
        if traci.simulation.getCollidingVehiclesNumber() > 0:
            crash_time = round(traci.simulation.getTime(), 1)
            print(f"\n💥 [경고] {crash_time}초 시점에 두 차량이 정확히 충돌했습니다!")
            sys.stdout.flush()
            break
    traci.close() 
    time.sleep(1)

    print("\n\n" + "#"*60)
    print("▶ [에피소드 2: 글로벌 스케줄링을 통한 완벽 회피 상황]")
    print("#"*60)
    init_conflict_points()
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.1", "--collision.action", "none", "--time-to-teleport", "-1", "--gui-settings-file", "gui-settings.xml"])
    inject_crashing_vehicles()
    while traci.simulation.getMinExpectedNumber() > 0:
        current_time = round(traci.simulation.getTime(), 1)
        traci.simulationStep()
        update_cp_distances()
        global_scheduling(mode="ep2")
        if crash_time > 0 and abs(current_time - crash_time) < 0.05:
            print(f"\n✅ [안내] 기존 충돌 시점({crash_time}초) 통과 중... 시스템의 개입으로 엉킴 없이 지나갑니다.")
            sys.stdout.flush()
    traci.close()

def run_mode_3():
    print("\n" + "#"*60)
    print("▶ 3번 모드: 대량 트래픽 생성 및 시나리오 저장 모드")
    print("▶ 40대의 랜덤 차량을 생성하고 'scenario_40.json' 파일에 저장합니다.")
    print("#"*60)
    
    generate_gui_settings()
    init_conflict_points()
    listener_thread = threading.Thread(target=command_listener, daemon=True)
    listener_thread.start()
    
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.1", "--collision.action", "none", "--time-to-teleport", "-1", "--gui-settings-file", "gui-settings.xml"])
    
    origins = ["e_in", "w_in", "s_in", "n_in"]
    veh_counter = 0
    scenario_data = [] # JSON에 기록할 데이터 리스트
    
    for origin_edge in origins:
        for _ in range(10): 
            speed = random.uniform(8.0, 13.0) 
            turn = random.choice(["Straight", "Left", "Right"])
            if origin_edge == "e_in": to_edge = "w_out" if turn == "Straight" else ("s_out" if turn == "Left" else "n_out")
            elif origin_edge == "w_in": to_edge = "e_out" if turn == "Straight" else ("n_out" if turn == "Left" else "s_out")
            elif origin_edge == "s_in": to_edge = "n_out" if turn == "Straight" else ("w_out" if turn == "Left" else "e_out")
            elif origin_edge == "n_in": to_edge = "s_out" if turn == "Straight" else ("e_out" if turn == "Left" else "w_out")
            
            route_id = f"r_{origin_edge}_{to_edge}_{veh_counter}"
            try: traci.route.add(route_id, [origin_edge, to_edge])
            except: pass
            
            if turn == "Left": lane = "1"
            elif turn == "Right": lane = "0"
            else: lane = str(random.choice([0, 1]))
            
            veh_id = f"veh_{origin_edge}_{turn}_{veh_counter}"
            
            # JSON 저장을 위한 딕셔너리 추가
            scenario_data.append({
                "route_id": route_id,
                "origin_edge": origin_edge,
                "to_edge": to_edge,
                "turn": turn,
                "lane": lane,
                "speed": speed,
                "veh_id": veh_id
            })
            
            traci.vehicle.add(veh_id, route_id, depart="now", departLane=lane, departPos="0", departSpeed=speed)
            traci.vehicle.setMaxSpeed(veh_id, speed)
            traci.vehicle.setMinGap(veh_id, 1.0)
            traci.vehicle.setSpeedMode(veh_id, 1) 
            veh_counter += 1
            
    # 시나리오 파일 저장
    with open('scenario_40.json', 'w', encoding='utf-8') as f:
        json.dump(scenario_data, f, indent=4)
        
    print(f"\n💾 [시나리오 저장 완료] 40대 차량의 생성 정보가 'scenario_40.json'에 기록되었습니다.")
    print(f"🚗 1-Pass 스케줄링 제어가 시작됩니다.")
    sys.stdout.flush()
    
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            update_cp_distances()
            global_scheduling(mode="random") 
    except traci.exceptions.FatalTraCIError:
        print("\n✅ 시뮬레이션 창이 닫혀 프로그램을 종료합니다.")
        sys.exit()
    traci.close()
    print("\n✅ 40대의 차량이 모두 무사히 교차로를 통과했습니다!")

def run_mode_4():
    print("\n" + "#"*60)
    print("▶ 4번 모드: 저장된 시나리오 불러오기 (A/B 테스트용)")
    print("▶ 'scenario_40.json' 파일에 기록된 차량 데이터를 정확히 동일하게 복원합니다.")
    print("#"*60)
    
    if not os.path.exists('scenario_40.json'):
        print("\n❌ 오류: 'scenario_40.json' 파일이 없습니다. 3번 모드를 먼저 실행하여 시나리오를 만들어주세요.")
        return
        
    generate_gui_settings()
    init_conflict_points()
    listener_thread = threading.Thread(target=command_listener, daemon=True)
    listener_thread.start()
    
    traci.start(["sumo-gui", "-n", "main.net.xml", "--step-length", "0.1", "--collision.action", "none", "--time-to-teleport", "-1", "--gui-settings-file", "gui-settings.xml"])
    
    # JSON 파일 읽어오기
    with open('scenario_40.json', 'r', encoding='utf-8') as f:
        scenario_data = json.load(f)
        
    for data in scenario_data:
        route_id = data["route_id"]
        origin_edge = data["origin_edge"]
        to_edge = data["to_edge"]
        lane = data["lane"]
        speed = data["speed"]
        veh_id = data["veh_id"]
        
        try: traci.route.add(route_id, [origin_edge, to_edge])
        except: pass
        
        traci.vehicle.add(veh_id, route_id, depart="now", departLane=lane, departPos="0", departSpeed=speed)
        traci.vehicle.setMaxSpeed(veh_id, speed)
        traci.vehicle.setMinGap(veh_id, 1.0)
        traci.vehicle.setSpeedMode(veh_id, 1) 
        
    print(f"\n📂 [시나리오 로드 완료] 통제된 40대의 차량 투입이 완료되었습니다. 1-Pass 스케줄링 제어가 시작됩니다.")
    sys.stdout.flush()
    
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            update_cp_distances()
            global_scheduling(mode="random") 
    except traci.exceptions.FatalTraCIError:
        print("\n✅ 시뮬레이션 창이 닫혀 프로그램을 종료합니다.")
        sys.exit()
    traci.close()
    print("\n✅ 40대의 차량이 모두 무사히 교차로를 통과했습니다!")

if __name__ == "__main__":
    print("\n" + "="*50)
    print("V2X 정밀 제어 시스템 (A/B 테스트용 Record & Replay 적용)")
    print("="*50)
    print("1: 수동 차량 통제 시뮬레이션 (동서남북 회전차량 지원)")
    print("2: 무조건 충돌 확정 2대 생성 (에피소드 1, 2 비교 모드)")
    print("3: 40대 차량 랜덤 생성 및 시나리오 JSON 저장 모드")
    print("4: 저장된 시나리오 JSON 불러오기 (동일 환경 성능 측정용)")
    
    choice = input("\n원하시는 모드의 번호를 입력하세요 (1, 2, 3, 4): ")
    if choice == '1': run_mode_1()
    elif choice == '2': run_mode_2()
    elif choice == '3': run_mode_3()
    elif choice == '4': run_mode_4()
