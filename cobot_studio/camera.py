"""Optional camera runtime and latest-frame mailbox; no serial access."""
import json
import shutil
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
from .mirror import HandSample

MODEL_URL = 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task'


def setup_runtime(root):
    root = Path(root)
    runtime = root/'.mirror-venv'
    python = runtime/'bin/python'
    if not python.exists():
        base = next((shutil.which(v) for v in ('python3.13','python3.12','python3.11') if shutil.which(v)),None)
        if not base:
            raise ValueError('Install Python 3.11–3.13 for the optional camera runtime.')
        subprocess.run([base,'-m','venv',str(runtime)],check=True,capture_output=True)
    subprocess.run([str(python),'-m','pip','install','mediapipe>=0.10.20,<0.11','opencv-contrib-python>=4.11,<6'],
                   check=True,capture_output=True)
    model = root/'.mirror-data/hand_landmarker.task'
    model.parent.mkdir(exist_ok=True)
    if not model.exists():
        temporary = model.with_suffix('.download')
        urllib.request.urlretrieve(MODEL_URL,temporary)
        temporary.replace(model)
    return 'Local camera runtime installed. Start camera when ready.'


class Camera:
    def __init__(self, root):
        self.root = Path(root)
        self.process = None
        self.latest = None
        self.error = None
        self.lock = threading.Lock()

    def start(self, index, rotation=0):
        if self.process is not None:
            raise ValueError('Stop the current camera first.')
        python = self.root/'.mirror-venv/bin/python'
        model = self.root/'.mirror-data/hand_landmarker.task'
        if not python.exists() or not model.exists():
            raise ValueError('Choose Install camera support first.')
        if rotation not in (0,90,180,270):
            raise ValueError('Camera rotation must be 0, 90, 180 or 270 degrees.')
        if type(index) is not int or not 0 <= index <= 10:
            raise ValueError('Camera number must be 0–10.')
        self.latest, self.error = None, None
        process = subprocess.Popen([str(python),'-m','cobot_studio.camera_worker',str(model),str(index),str(rotation)],
                                   cwd=self.root,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        self.process = process
        def read():
            try:
                for line in process.stdout:
                    if not line.lstrip().startswith('{'):
                        continue
                    packet = json.loads(line)
                    with self.lock:
                        if self.process is not process:
                            break
                        if packet.get('kind') == 'error':
                            self.error = packet['message']
                        else:
                            self.latest = packet
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
            finally:
                process.stdout.close()
                with self.lock:
                    if self.process is process and not self.error:
                        self.error = 'Camera stopped. Restart and recalibrate.'
        threading.Thread(target=read,daemon=True).start()

    def packet(self):
        with self.lock:
            return self.latest, self.error

    def sample(self):
        packet,error = self.packet()
        if error or not packet or packet.get('features') is None:
            raise ValueError(error or 'Show one clear hand to the camera.')
        sample = HandSample(packet['captured_at'],tuple(packet['features']),packet['confidence'],packet['handedness'],tuple(packet.get('extensions',())),packet.get('aspect_ratio',4/3))
        if not sample.valid(time.monotonic()):
            raise ValueError('Tracking stale or uncertain. Recalibrate with a clear palm.')
        return sample

    def stop(self):
        process, self.process = self.process, None
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
        with self.lock:
            self.latest = None
            self.error = None
