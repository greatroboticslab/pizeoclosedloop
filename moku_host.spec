# Build natively on each target OS with Python 3.13.
import sys
from pathlib import Path

from PyInstaller.utils.hooks import collect_data_files, collect_submodules

project = Path(SPECPATH)
datas = [(str(project / "configs"), "configs")]
datas += collect_data_files("moku")

a = Analysis(
    [str(project / "app.py")],
    pathex=[str(project)],
    binaries=[],
    datas=datas,
    hiddenimports=collect_submodules("moku.instruments"),
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tkinter", "ttkbootstrap", "paho", "matplotlib", "pytest", "pyqtgraph.opengl"],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="LaserAI",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name="LaserAI",
)
if sys.platform == "darwin":
    app = BUNDLE(
        coll,
        name="LaserAI.app",
        icon=None,
        bundle_identifier="org.greatroboticslab.laserai",
        info_plist={"NSHighResolutionCapable": True},
    )
