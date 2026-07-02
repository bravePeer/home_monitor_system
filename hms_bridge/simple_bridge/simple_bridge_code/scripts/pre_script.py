import os
import sys
import subprocess

def ensure(pkg):
    try:
        __import__(pkg)
    except ImportError:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", pkg
        ])

ensure("python-dotenv")

from dotenv import load_dotenv
from SCons.Script import Import
Import("env")

print("Run: pre_script")

load_dotenv()

defines_to_add = [
    "RECEIVER_WIFI_SSID",
    "RECEIVER_WIFI_PASSWORD",
    "RECEIVER_HMS_SERVER_API_URL"
]

cpp_defines = []
for define in defines_to_add:
    cpp_defines.append((define, f'\\"{os.getenv(define)}\\"'))

env.Append(CPPDEFINES=cpp_defines)

def compress_website_action(*args, **kwargs):
    print("Compressing website files for filesystem build")
    from pathlib import Path
    sys.path.insert(0, str(Path(".\scripts")))
    import compress_website as compress_website
    os.makedirs("data", exist_ok=True)
    compress_website.compress_website(dest_dir="data", ignore_files=["app.py"])

def upload_filesystem_action(*args, **kwargs):
    pass

env.AddCustomTarget(
    name="simple_bridge_compress_website",
    dependencies=None,
    actions=compress_website_action,
    title="Compressing website files",
    description="Compress website files for embedded system"
)

env.AddCustomTarget(
    name="simple_bridge_upload_filesystem",
    dependencies="simple_bridge_compress_website",
    actions=upload_filesystem_action,
    title="Uploading filesystem",
    description="Upload filesystem to embedded system"
)

env.AddPreAction("buildfs", compress_website_action)
