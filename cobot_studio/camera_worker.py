"""Optional isolated MediaPipe/OpenCV worker. Only stdout JSON crosses into Tk.

Video stays local, is not recorded, and is only captured after Start camera.
"""
import base64
import json
import sys
import time
from pathlib import Path


def main():
    import cv2
    import mediapipe as mp
    from .mirror import hand_features
    model, camera = sys.argv[1], int(sys.argv[2])
    rotation = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    cap = None
    try:
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1, min_hand_detection_confidence=0.8,
            min_hand_presence_confidence=0.8, min_tracking_confidence=0.8)
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as detector:
            cap = cv2.VideoCapture(camera, cv2.CAP_AVFOUNDATION if sys.platform == 'darwin' else cv2.CAP_ANY)
            if not cap.isOpened():
                raise ValueError('Camera unavailable. Check camera number and macOS Camera permission.')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
            previous = -1
            while True:
                ok, frame = cap.read()
                captured = time.monotonic()
                if not ok:
                    raise ValueError('Camera frame unavailable.')
                if rotation:
                    codes = {90:cv2.ROTATE_90_CLOCKWISE,180:cv2.ROTATE_180,270:cv2.ROTATE_90_COUNTERCLOCKWISE}
                    frame = cv2.rotate(frame,codes[rotation])
                rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
                stamp = max(previous+1,int(captured*1000))
                previous = stamp
                result = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB,data=rgb),stamp)
                packet = {'kind':'frame', 'captured_at':captured, 'confidence':0, 'handedness':'', 'features':None}
                if result.hand_landmarks:
                    points = [(p.x,p.y,p.z) for p in result.hand_landmarks[0]]
                    world = [(p.x,p.y,p.z) for p in result.hand_world_landmarks[0]]
                    try:
                        packet['features'] = hand_features(points,world)
                        category = result.handedness[0][0]
                        packet.update(confidence=category.score,handedness=category.category_name)
                        for x,y,_ in points:
                            cv2.circle(frame,(int(x*frame.shape[1]),int(y*frame.shape[0])),3,(80,230,100),-1)
                    except ValueError:
                        pass
                # Unflipped image: mapping and camera display use the same convention.
                small = cv2.cvtColor(cv2.resize(frame,(320,240)),cv2.COLOR_BGR2RGB)
                ppm = b'P6\n320 240\n255\n'+small.tobytes()
                packet['ppm'] = base64.b64encode(ppm).decode('ascii')
                print(json.dumps(packet),flush=True)
    except Exception as exc:
        print(json.dumps({'kind':'error','message':str(exc)}),flush=True)
    finally:
        if cap is not None:
            cap.release()


if __name__ == '__main__':
    main()
