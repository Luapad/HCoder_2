import os
import runpy
import traci
import xml.etree.ElementTree as ET

# =========================================================
# 1. traci.start 함수 가로채기 (Monkey Patching)
# =========================================================
original_start = traci.start


def modified_start(cmd, *args, **kwargs):
    print("\n[시스템] ⛽ SUMO 배출가스/연료 추적 옵션을 안전하게 주입합니다...")
    cmd_list = list(cmd)
    if "--emission-output" not in cmd_list:
        cmd_list.extend(["--emission-output", "fuel_output.xml"])
    return original_start(cmd_list, *args, **kwargs)


traci.start = modified_start


# =========================================================
# 2. 총 연료 소모량 계산 로직 (특정 차량 필터링 추가)
# =========================================================
def calculate_fuel(target_veh_id=None):
    if not os.path.exists("fuel_output.xml"):
        print("❌ 연료 출력 파일(fuel_output.xml)을 찾을 수 없습니다.")
        return

    if target_veh_id:
        print(f"\n[시스템] 📊 🎯 특정 차량 ['{target_veh_id}']의 연료 소모량 계산 중...")
    else:
        print("\n[시스템] 📊 🌐 전체 차량의 총 연료 소모량 계산 중...")

    tree = ET.parse("fuel_output.xml")
    root = tree.getroot()

    total_fuel_mg = 0.0
    step_length = 0.01  # 원본 코드의 STEP_LENGTH와 동일
    found_vehicle = False

    for timestep in root.findall('timestep'):
        for vehicle in timestep.findall('vehicle'):
            vid = vehicle.get('id')

            if target_veh_id and vid != target_veh_id:
                continue

            if target_veh_id and vid == target_veh_id:
                found_vehicle = True

            fuel_rate = float(vehicle.get('fuel', 0))
            total_fuel_mg += fuel_rate * step_length

    if target_veh_id and not found_vehicle:
        print(f"⚠️ 경고: XML 파일에서 '{target_veh_id}' 차량을 찾을 수 없습니다. 아이디를 확인해주세요.")
        return

    total_fuel_g = total_fuel_mg / 1000.0
    total_fuel_ml = total_fuel_g / 0.75

    target_name = f"차량 '{target_veh_id}'" if target_veh_id else "전체 차량"

    print(f"=========================================")
    print(f"✅ {target_name} 연료 소모량: 약 {total_fuel_g:.4f} g")
    print(f"✅ 부피 환산(휘발유)   : 약 {total_fuel_ml:.4f} mL")
    print(f"=========================================\n")


# =========================================================
# 3. 원본 스크립트 실행 및 결과 도출
# =========================================================
if __name__ == "__main__":
    target_file = "FIFO.py"  # 📌 실행할 원본 파일명

    # 🎯 여기에 추적하고 싶은 차량 ID를 적어주세요.
    # 전체 차량을 보고 싶다면 None 으로 변경하세요. (TARGET_VEHICLE_ID = None)
    TARGET_VEHICLE_ID = "veh_w_in_Left_17"

    if not os.path.exists(target_file):
        print(f"❌ '{target_file}' 파일이 같은 폴더에 없습니다. 파일명을 확인해주세요.")
    else:
        if TARGET_VEHICLE_ID:
            print(f"\n💡 [설정됨] 특정 차량 '{TARGET_VEHICLE_ID}'의 연료 소모량만 추적합니다.")
        else:
            print("\n💡 [설정됨] 전체 차량의 연료 소모량을 추적합니다.")

        print(f"▶ '{target_file}' 실행을 시작합니다...")

        try:
            runpy.run_path(target_file, run_name="__main__")
        except Exception as e:
            print(f"\n❌ 실행 중 에러 발생: {e}")
        finally:
            calculate_fuel(target_veh_id=TARGET_VEHICLE_ID)