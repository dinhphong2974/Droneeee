"""
Standalone test: Verify map.html loads correctly in QWebEngineView.
No connection dialog — just loads the map and prints console output.
Auto-exits after 6 seconds.
"""
import sys, os
sys.stdout.reconfigure(encoding='utf-8')

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer, QUrl
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWebEngineCore import QWebEngineSettings, QWebEnginePage
from PySide6.QtWebChannel import QWebChannel

class TestPage(QWebEnginePage):
    """Custom page that captures JS console messages."""
    def javaScriptConsoleMessage(self, level, msg, line, source):
        print(f"  JS [{level}]: {msg}")

app = QApplication(sys.argv)
win = QMainWindow()
win.setWindowTitle("Map Test")
win.resize(800, 600)

view = QWebEngineView()
page = TestPage(view)
view.setPage(page)

# Settings
settings = view.settings()
settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, True)
settings.setAttribute(QWebEngineSettings.WebAttribute.JavascriptEnabled, True)

# WebChannel (same as mission_tab.py)
from PySide6.QtCore import QObject, Signal, Slot
class TestBridge(QObject):
    map_clicked = Signal(float, float)
    @Slot(float, float)
    def on_map_clicked(self, lat, lon):
        print(f"  BRIDGE: Map clicked at ({lat}, {lon})")

bridge = TestBridge()
channel = QWebChannel()
channel.registerObject("bridge", bridge)
page.setWebChannel(channel)

def on_load_finished(ok):
    print(f"[TEST] loadFinished: ok={ok}")
    if ok:
        # Test runJavaScript after load
        view.page().runJavaScript(
            "typeof L !== 'undefined' ? 'Leaflet OK (v' + L.version + ')' : 'Leaflet MISSING'",
            lambda result: print(f"[TEST] Leaflet check: {result}")
        )
        view.page().runJavaScript(
            "window.droneMap ? 'Map instance OK' : 'Map MISSING'",
            lambda result: print(f"[TEST] Map instance: {result}")
        )
        view.page().runJavaScript(
            "typeof updateDronePosition === 'function' ? 'updateDronePosition OK' : 'MISSING'",
            lambda result: print(f"[TEST] updateDronePosition: {result}")
        )
        view.page().runJavaScript(
            "typeof updateWaypoints === 'function' ? 'updateWaypoints OK' : 'MISSING'",
            lambda result: print(f"[TEST] updateWaypoints: {result}")
        )
        # Test calling updateDronePosition
        view.page().runJavaScript(
            "updateDronePosition(21.0285, 105.8542, 45); 'Drone marker created'",
            lambda result: print(f"[TEST] Drone marker: {result}")
        )

view.loadFinished.connect(on_load_finished)

# Load map.html
map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'assets', 'map.html'))
print(f"[TEST] Loading: {map_path}")
print(f"[TEST] File exists: {os.path.isfile(map_path)}")
view.setUrl(QUrl.fromLocalFile(map_path))

win.setCentralWidget(view)
win.show()

# Auto-exit after 6 seconds
QTimer.singleShot(6000, lambda: (print("[TEST] === ALL TESTS COMPLETE ==="), app.quit()))

sys.exit(app.exec())
