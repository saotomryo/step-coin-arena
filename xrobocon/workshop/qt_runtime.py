"""Keep Qt plugin discovery working when macOS marks venv dylibs hidden.

Qt's directory scanner ignores UF_HIDDEN files even though dlopen can load them.
Use a small process-local copy, without modifying the installed environment.
"""
import shutil
import stat
import tempfile
from pathlib import Path
from PySide6.QtCore import QCoreApplication, QLibraryInfo

_plugin_directory = None

def prepare_plugins():
    global _plugin_directory
    source=Path(QLibraryInfo.path(QLibraryInfo.LibraryPath.PluginsPath))
    platforms=list((source/'platforms').glob('*.dylib'))
    if _plugin_directory is not None or not any(p.stat().st_flags & getattr(stat,'UF_HIDDEN',0) for p in platforms):return
    _plugin_directory=tempfile.TemporaryDirectory(prefix='xrobocon-qt-plugins-')
    root=Path(_plugin_directory.name)
    for group in ('platforms','styles','platformthemes'):
        if not (source/group).exists():continue
        (root/group).mkdir()
        for file in (source/group).glob('*.dylib'):
            # copyfile deliberately excludes Finder flags/xattrs.
            shutil.copyfile(file,root/group/file.name)
    QCoreApplication.addLibraryPath(str(root))
