import threading
import os
import sys

import rospy


def _append_conda_site_packages():
    version_tag = "python{}.{}".format(sys.version_info[0], sys.version_info[1])
    candidates = []

    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix:
        candidates.append(os.path.join(conda_prefix, "lib", version_tag, "site-packages"))

    home_dir = os.path.expanduser("~")
    candidates.append(os.path.join(home_dir, "miniconda3", "envs", "shibie", "lib", version_tag, "site-packages"))

    for candidate in candidates:
        if os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)


_append_conda_site_packages()

try:
    import pyttsx3  # type: ignore

    _HAS_PYTTSX3 = True
except Exception:
    pyttsx3 = None
    _HAS_PYTTSX3 = False


_SPEECH_ENGINE = None
_SPEECH_LOCK = threading.Lock()
_SPEECH_VOICE_ID = None
_SPEECH_RATE = 100


def _reset_speech_engine():
    global _SPEECH_ENGINE
    try:
        if _SPEECH_ENGINE is not None:
            _SPEECH_ENGINE.stop()
    except Exception:
        pass
    _SPEECH_ENGINE = None


def _get_speech_engine():
    global _SPEECH_ENGINE, _SPEECH_VOICE_ID
    if _SPEECH_ENGINE is None:
        _SPEECH_ENGINE = pyttsx3.init()
        voices = _SPEECH_ENGINE.getProperty("voices")
        if len(voices) > 0:
            preferred = [
                ("English (America)", "en-us"),
                ("English (Received Pronunciation)", None),
                ("English (Great Britain)", None),
            ]
            selected = None
            for name_key, id_key in preferred:
                for voice in voices:
                    try:
                        voice_name = voice.name or ""
                        voice_id = voice.id or ""
                    except Exception:
                        voice_name = ""
                        voice_id = ""
                    if name_key and name_key in voice_name:
                        selected = voice
                        break
                    if id_key and id_key.lower() in voice_id.lower():
                        selected = voice
                        break
                if selected is not None:
                    break
            if selected is None:
                selected = voices[0]
            _SPEECH_VOICE_ID = selected.id
            _SPEECH_ENGINE.setProperty("voice", _SPEECH_VOICE_ID)
        _SPEECH_ENGINE.setProperty("rate", _SPEECH_RATE)
    return _SPEECH_ENGINE


def speak(data):
    if not _HAS_PYTTSX3:
        rospy.loginfo("speak: %s", data)
        return True, "pyttsx3 unavailable; logged text only"
    with _SPEECH_LOCK:
        for _ in range(2):
            try:
                speech = _get_speech_engine()
                speech.say(data)
                speech.runAndWait()
                return True, "spoken"
            except ReferenceError:
                _reset_speech_engine()
                continue
            except Exception as exc:
                rospy.logwarn("speech failed: %s", exc)
                break
    return False, "speech synthesis failed"
