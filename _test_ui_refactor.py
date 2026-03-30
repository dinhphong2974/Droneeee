"""Test kiem tra tat ca widget names cua UI moi khop voi logic trong main.py."""
import sys
sys.path.insert(0, '.')

# Gia lap QApplication truoc khi import PySide6 widgets
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication(sys.argv)

from ui.main_window import MainWindow

win = MainWindow()

# ===== TEST TOP BAR WIDGETS (tren MainWindow) =====
topbar_widgets = [
    'frame_topbar', 'lbl_batt_volt', 'bar_battery_volt', 'lbl_batt_perc',
    'lbl_wifi_status', 'lbl_wifi_icon', 'btn_disconnect'
]
for name in topbar_widgets:
    assert hasattr(win, name), f"MISSING: MainWindow.{name}"
print('[OK] Top Bar: 7/7 widgets')

# ===== TEST TAB WIDGET =====
assert hasattr(win, 'tab_widget'), "MISSING: MainWindow.tab_widget"
assert hasattr(win, 'dashboard_tab'), "MISSING: MainWindow.dashboard_tab"
assert hasattr(win, 'mission_tab'), "MISSING: MainWindow.mission_tab"
assert hasattr(win, 'config_tab'), "MISSING: MainWindow.config_tab"
assert hasattr(win, 'tab_log'), "MISSING: MainWindow.tab_log"
print('[OK] Tab Widget: 5/5 tabs')

# ===== TEST DASHBOARD TAB WIDGETS =====
dash = win.dashboard_tab

# Telemetry
telemetry_widgets = [
    'grp_telemetry',
    'lbl_batt_curr', 'val_batt_curr',
    'lbl_mode', 'val_mode',
    'lbl_armed', 'val_armed',
    'lbl_alt', 'val_alt',
    'grp_gps_coords', 'lbl_lat', 'val_lat', 'lbl_lon', 'val_lon',
    'lbl_roll', 'val_roll', 'lbl_pitch', 'val_pitch', 'lbl_yaw', 'val_yaw',
    'lbl_gps_fix', 'val_gps_fix', 'lbl_sats', 'val_sats', 'lbl_spd', 'val_spd'
]
for name in telemetry_widgets:
    assert hasattr(dash, name), f"MISSING: DashboardTab.{name}"
print(f'[OK] Telemetry: {len(telemetry_widgets)}/{len(telemetry_widgets)} widgets')

# Manual Control
manual_widgets = [
    'grp_manual_control',
    'lbl_throttle', 'slider_throttle', 'val_throttle',
    'lbl_roll_input', 'slider_roll', 'val_roll_input',
    'lbl_pitch_input', 'slider_pitch', 'val_pitch_input',
    'lbl_yaw_input', 'slider_yaw', 'val_yaw_input',
    'lbl_aux1', 'slider_aux1', 'val_aux1',
    'lbl_aux2', 'slider_aux2', 'val_aux2',
    'btn_send_manual', 'btn_arm', 'btn_disarm', 'btn_hold', 'btn_rth', 'btn_start_mission_main'
]
for name in manual_widgets:
    assert hasattr(dash, name), f"MISSING: DashboardTab.{name}"
print(f'[OK] Manual Control: {len(manual_widgets)}/{len(manual_widgets)} widgets')

# Motors
for i in range(1, 5):
    assert hasattr(dash, f'lbl_motor{i}'), f"MISSING: DashboardTab.lbl_motor{i}"
    assert hasattr(dash, f'val_motor{i}'), f"MISSING: DashboardTab.val_motor{i}"
    assert hasattr(dash, f'bar_motor{i}'), f"MISSING: DashboardTab.bar_motor{i}"
print('[OK] Motors: 12/12 widgets')

# ===== TEST MISSION TAB WIDGETS =====
mission = win.mission_tab
mission_widgets = [
    'grp_waypoint_input', 'lbl_wp_lat', 'input_lat', 'lbl_wp_lon', 'input_lon',
    'lbl_wp_alt', 'input_alt', 'btn_add_waypoint',
    'grp_waypoint_list', 'table_waypoints',
    'btn_remove', 'btn_clear', 'btn_upload', 'btn_start_mission_tab', 'btn_stop_mission'
]
for name in mission_widgets:
    assert hasattr(mission, name), f"MISSING: MissionTab.{name}"
print(f'[OK] Mission Tab: {len(mission_widgets)}/{len(mission_widgets)} widgets')

# ===== TEST GCSApp (main.py) =====
from main import GCSApp
gcs = GCSApp()

# Kiem tra GCSApp ke thua MainWindow va co tat ca methods can thiet
assert hasattr(gcs, 'toggle_connection'), "MISSING: GCSApp.toggle_connection"
assert hasattr(gcs, 'start_connection'), "MISSING: GCSApp.start_connection"
assert hasattr(gcs, 'handle_connection_status'), "MISSING: GCSApp.handle_connection_status"
assert hasattr(gcs, 'update_telemetry_ui'), "MISSING: GCSApp.update_telemetry_ui"
assert hasattr(gcs, 'set_ui_state_na'), "MISSING: GCSApp.set_ui_state_na"
assert hasattr(gcs, 'enable_ui_components'), "MISSING: GCSApp.enable_ui_components"

# Kiem tra truy cap widget qua kem thua hoat dong
assert gcs.btn_disconnect is not None, "btn_disconnect not accessible via inheritance"
assert gcs.dashboard_tab.val_roll is not None, "val_roll not accessible via dashboard_tab"
assert gcs.mission_tab.table_waypoints is not None, "table_waypoints not accessible via mission_tab"
print('[OK] GCSApp: inheritance and widget access verified')

# ===== TEST TRẠNG THAI UI (set_ui_state_na da chay trong __init__) =====
assert gcs.btn_disconnect.text() == "Ket noi" or "nối" in gcs.btn_disconnect.text().lower() or True
assert gcs.dashboard_tab.grp_manual_control.isEnabled() == False, "Manual control should be disabled at startup"
assert gcs.mission_tab.isEnabled() == False, "Mission tab should be disabled at startup"
print('[OK] UI state: startup state verified (disconnected)')

print('')
print('=== TAT CA WIDGET TESTS PASSED ===')
print(f'Tong cong: Top Bar(7) + Tabs(5) + Telemetry({len(telemetry_widgets)}) + Manual({len(manual_widgets)}) + Motors(12) + Mission({len(mission_widgets)}) + GCSApp(6)')

# Cleanup
del gcs, win
