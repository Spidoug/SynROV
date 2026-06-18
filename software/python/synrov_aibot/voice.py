"""Sistema de voz e áudio do SynROV AiBot."""
from __future__ import annotations

import base64
import json
import math
import os
import queue
import struct
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from .helpers import normalize_text
from .robot_ai import DRONE_PROFILE, MANIPULATOR_PROFILE, VEHICLE_PROFILE, match_alias


AUDIO_KEYS: List[str] = ["rms", "peak", "zcr", "centroid", "low", "mid", "high", "pulse"]
_AUDIO_DIM = len(AUDIO_KEYS)


try:
    import speech_recognition as _sr_lib
    _SR_AVAILABLE = True
    _SR_IMPORT_ERROR = ""
except Exception as _sr_exc:
    _sr_lib = None
    _SR_AVAILABLE = False
    _SR_IMPORT_ERROR = str(_sr_exc)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        v = float(value)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def _parse_device_index(device: Any) -> Optional[int]:
    """Converte índice de dispositivo para int ou None (padrão do sistema)."""
    if device is None:
        return None
    if isinstance(device, int):
        return device if device >= 0 else None
    text = str(device).strip().lower()
    if text in ("", "auto", "default", "none", "-1"):
        return None
    try:
        idx = int(text)
        return idx if idx >= 0 else None
    except Exception:
        return None


def _safe_device_label(device: Any) -> str:
    """Retorna uma descrição legível do dispositivo selecionado."""
    idx = _parse_device_index(device)
    if idx is None:
        return "padrão do sistema"
    if _SR_AVAILABLE:
        try:
            import pyaudio as _pa
            p = _pa.PyAudio()
            try:
                info = p.get_device_info_by_index(idx)
                name = str(info.get("name", f"dispositivo {idx}"))
                return f"{idx} · {name}"
            finally:
                p.terminate()
        except Exception:
            pass
    return f"dispositivo {idx}"


_METER_SCRIPT = r"""
from __future__ import annotations

import array, json, math, os, struct, sys
from typing import Tuple

FRAMES = 1024

raw_dev = os.environ.get("SYNROV_AUDIO_DEVICE_INDEX", "").strip()
try:
    selected = int(raw_dev) if raw_dev not in ("", "auto", "default", "None") else None
except Exception:
    selected = None


def emit(obj):
    sys.stdout.write(json.dumps(obj, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def rms_peak(data: bytes, fmt: str) -> Tuple[float, float]:
    if not data:
        return 0.0, 0.0
    if fmt == "float32":
        n = len(data) // 4
        if n <= 0:
            return 0.0, 0.0
        vals = struct.unpack_from("<%df" % n, data[: n * 4])
        s2 = peak = 0.0
        for v in vals:
            fv = float(v) if math.isfinite(float(v)) else 0.0
            av = abs(fv)
            if av > peak:
                peak = av
            s2 += fv * fv
        return math.sqrt(s2 / max(1, n)), peak
    arr = array.array("h")
    arr.frombytes(data[: len(data) - (len(data) % 2)])
    if sys.byteorder != "little":
        arr.byteswap()
    if not arr:
        return 0.0, 0.0
    s2 = peak = 0.0
    scale = 32768.0
    for sample in arr:
        v = float(sample) / scale
        av = abs(v)
        if av > peak:
            peak = av
        s2 += v * v
    return math.sqrt(s2 / len(arr)), peak


try:
    import pyaudio
except Exception as exc:
    sys.stderr.write("PyAudio ausente: %s\n" % exc)
    sys.stderr.flush()
    sys.exit(2)

pa = pyaudio.PyAudio()
stream = None
opened = None

try:
    candidates = []
    if selected is not None:
        candidates.append(selected)
    candidates.append(None)  # padrão do sistema
    try:
        for idx in range(pa.get_device_count()):
            try:
                info = pa.get_device_info_by_index(idx)
                if int(info.get("maxInputChannels", 0) or 0) > 0 and idx not in candidates:
                    candidates.append(idx)
            except Exception:
                pass
    except Exception:
        pass

    last_err = ""
    for dev_idx in candidates:
        for rate in (16000, 44100, 48000, 22050, 8000):
            for fmt_name, fmt in (("int16", pyaudio.paInt16), ("float32", pyaudio.paFloat32)):
                try:
                    kw = dict(format=fmt, channels=1, rate=rate, input=True,
                              frames_per_buffer=FRAMES)
                    if dev_idx is not None:
                        kw["input_device_index"] = dev_idx
                    stream = pa.open(**kw)
                    opened = (dev_idx, rate, fmt_name)
                    break
                except Exception as exc:
                    last_err = str(exc)
                    stream = None
            if opened:
                break
        if opened:
            break

    if not opened:
        sys.stderr.write("Falha ao abrir microfone: %s\n" % last_err)
        sys.stderr.flush()
        sys.exit(3)

    dev_index, rate, fmt_name = opened
    dev_name = "default"
    try:
        info = (pa.get_device_info_by_index(dev_index)
                if dev_index is not None
                else pa.get_default_input_device_info())
        dev_name = str(info.get("name", dev_name))
        if dev_index is None:
            dev_index = int(info.get("index", -1))
    except Exception:
        pass

    emit({"ok": 1, "device_index": dev_index, "device_name": dev_name,
          "rate": rate, "format": fmt_name})

    prev_rms = 0.0
    while True:
        data = stream.read(FRAMES, exception_on_overflow=False)
        rms, peak = rms_peak(data, fmt_name)
        pulse = max(-1.0, min(1.0, (rms - prev_rms) * 12.0))
        prev_rms = rms
        emit({"r": rms, "p": peak, "u": pulse})

except Exception as exc:
    sys.stderr.write(str(exc) + "\n")
    sys.stderr.flush()
    sys.exit(4)
finally:
    try:
        if stream is not None:
            stream.stop_stream()
            stream.close()
    except Exception:
        pass
    try:
        pa.terminate()
    except Exception:
        pass
"""


_METER_RUNNER = (
    "import base64; exec(compile(base64.b64decode(%r).decode('utf-8'), "
    "'<synrov_audio_meter>', 'exec'))\n"
) % base64.b64encode(_METER_SCRIPT.encode("utf-8")).decode("ascii")


class RichVoiceParser:
    """Parser de intenção backed pelos três perfis de robô do SynROV."""

    def __init__(self) -> None:
        self._profiles = [MANIPULATOR_PROFILE, VEHICLE_PROFILE, DRONE_PROFILE]

    def parse(self, text: Any, robot: str = "") -> Tuple[str, float]:
        """Retorna (intent_name, confidence) para o texto dado."""
        best_score = 0.0
        best_intent = "none"
        for profile in self._profiles:
            for name, aliases in profile.intents.items():
                score = match_alias(text, aliases)
                if score > best_score:
                    best_score = score
                    best_intent = name
            for name, aliases in profile.missions.items():
                score = match_alias(text, aliases)
                if score > best_score:
                    best_score = score
                    best_intent = name
        conf = min(1.0, best_score / 1000.0)
        return best_intent, conf

    def parse_full(self, text: Any) -> Dict[str, Any]:
        """Retorna dicionário completo com texto, norma, robô, tipo, intenção e confiança."""
        best = (0.0, "Manipulator", "none", "none")
        for profile in self._profiles:
            for name, aliases in profile.intents.items():
                score = match_alias(text, aliases)
                if score > best[0]:
                    best = (score, profile.robot, "intent", name)
            for name, aliases in profile.missions.items():
                score = match_alias(text, aliases)
                if score > best[0]:
                    best = (score, profile.robot, "mission", name)
        return {
            "text": str(text or ""),
            "norm": normalize_text(text),
            "robot": best[1],
            "kind": best[2],
            "intent": best[3],
            "conf": min(1.0, best[0] / 1000.0),
        }


class GoogleVoiceAudioEngine:
    """Engine completo de voz e áudio para o SynROV AiBot.

    Funcionalidades:
    - Medição de nível de áudio em subprocesso isolado (não bloqueia a UI)
    - Reconhecimento de fala via Google Speech Recognition
    - Suporte a múltiplos idiomas com getter dinâmico
    - Features de áudio (RMS, peak, pulse) para o pipeline de aprendizado
    - Contexto rico com backend, diagnóstico e estado

    A captura de áudio e o reconhecimento rodam em threads daemon separadas.
    O start() retorna imediatamente e nunca bloqueia o Tkinter.
    """

    def __init__(
        self,
        parser: RichVoiceParser,
        robot_getter: Any,
        language_getter: Optional[Any] = None,
    ) -> None:
        self.parser = parser
        self.robot_getter = robot_getter
        self.language_getter: Any = language_getter or (lambda: "pt")


        self.running = False
        self.last_text = ""
        self.last_norm = ""
        self.last_intent = "none"
        self.last_conf = 0.0
        self.last_text_ts = 0.0
        self.last_sr_error = ""
        self.status_message = ""
        self._partial_text = ""


        self.audio_level = 0.0
        self.audio_peak = 0.0
        self.audio_meter = 0.0
        self.audio_features: List[float] = [0.0] * _AUDIO_DIM
        self.audio_ts = 0.0


        self.device: Optional[Any] = os.environ.get("SYNROV_VOICE_DEVICE", "") or None
        self.device_label: str = _safe_device_label(self.device)


        self.audio_backend = "off"
        self.speech_backend = "google_idle"


        self._audio_lock = threading.Lock()
        self._state_lock = threading.RLock()


        self._audio_thread: Optional[threading.Thread] = None
        self._speech_thread: Optional[threading.Thread] = None
        self._recognition_thread: Optional[threading.Thread] = None
        self._recognition_busy = False


        self._meter_proc: Optional[subprocess.Popen] = None


        self._noise_floor = 0.0035


    def set_device(self, device: Any) -> None:
        """Define o índice do dispositivo de áudio a usar."""
        self.device = device
        self.device_label = _safe_device_label(device)

    def set_language_getter(self, language_getter: Any) -> None:
        """Registra um callable que retorna o idioma atual (ex: lambda: lang_var.get())."""
        if callable(language_getter):
            self.language_getter = language_getter


    def start(self) -> bool:
        """Inicia captura de áudio e reconhecimento de fala em threads daemon.

        Nunca bloqueia. Retorna True se já estava rodando ou iniciou com sucesso.
        """
        if self.running:
            return True

        self.last_sr_error = ""
        self.status_message = ""
        self.device_label = _safe_device_label(self.device)

        self.audio_backend = "starting_meter"
        self.speech_backend = (
            "starting_google" if _SR_AVAILABLE else "speech_missing_dependency"
        )

        with self._audio_lock:
            self.audio_level = 0.0
            self.audio_peak = 0.0
            self.audio_meter = 0.0
            self.audio_features = [0.0] * _AUDIO_DIM
            self.audio_ts = 0.0

        self.running = True


        if self._audio_thread is None or not self._audio_thread.is_alive():
            self._audio_thread = threading.Thread(
                target=self._pyaudio_meter_loop,
                name="synrov-audio-meter",
                daemon=True,
            )
            self._audio_thread.start()


        if not _SR_AVAILABLE:
            self.last_sr_error = (
                "SpeechRecognition não instalado. Barra de som ativa; "
                "voz por texto via inject_text(). "
                f"Para comandos de voz: pip install SpeechRecognition PyAudio. "
                f"Erro original: {_SR_IMPORT_ERROR}"
            )
            self.status_message = self.last_sr_error
            return True

        if self._speech_thread is None or not self._speech_thread.is_alive():
            self._speech_thread = threading.Thread(
                target=self._speech_loop,
                name="synrov-google-speech",
                daemon=True,
            )
            self._speech_thread.start()

        return True

    def stop(self) -> None:
        """Para captura de áudio e reconhecimento de fala."""
        self.running = False


        proc = self._meter_proc
        try:
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.2)
                except subprocess.TimeoutExpired:
                    proc.kill()
        except Exception:
            pass
        self._meter_proc = None


        with self._audio_lock:
            self.audio_level = 0.0
            self.audio_peak = 0.0
            self.audio_meter = 0.0
            self.audio_features = [0.0] * _AUDIO_DIM

        self.speech_backend = "google_idle"
        self.audio_backend = "off"
        self.status_message = "Microfone parado."


    def inject_text(self, text: str) -> None:
        """Injeta texto diretamente como se tivesse sido reconhecido pelo microfone."""
        text = str(text or "").strip()
        if not text:
            return
        robot = ""
        try:
            robot = str(self.robot_getter() or "")
        except Exception:
            pass
        intent, conf = self.parser.parse(text, robot)
        self.last_text = text
        self.last_norm = normalize_text(text)
        self.last_intent = intent
        self.last_conf = conf
        self.last_text_ts = time.time()
        self._partial_text = ""


    def context(self) -> Dict[str, Any]:
        """Retorna o estado completo do sistema de voz e áudio."""
        with self._audio_lock:
            feats = list(self.audio_features)
            lvl = self.audio_level
            peak = self.audio_peak
            meter = self.audio_meter
            ats = self.audio_ts

        diag = self.status_message or self.last_sr_error
        now = time.time()

        return {

            "raw": self.last_text,
            "text": self.last_text,
            "norm": self.last_norm,
            "intent": self.last_intent,
            "conf": self.last_conf,
            "text_age": now - self.last_text_ts if self.last_text_ts else 999.0,

            "audio_features": feats,
            "audio_level": lvl,
            "audio_peak": peak,
            "audio_meter": _clamp(_safe_float(meter), 0.0, 1.0),
            "audio_age": now - ats if ats else 999.0,

            "error": self.last_sr_error,
            "audio_backend": self.audio_backend,
            "speech_backend": self.speech_backend,
            "diagnostics": diag,
            "partial": self._partial_text,
            "offline": False,
            "model_path": "",
            "device_label": self.device_label,
        }


    def _recognition_languages(self) -> List[str]:
        try:
            lang = str(self.language_getter() or "").strip().lower()
        except Exception:
            lang = "pt"
        primary = "pt-BR" if lang.startswith("pt") else "en-US"
        fallback = "en-US" if primary == "pt-BR" else "pt-BR"
        return [primary, fallback]


    def _pyaudio_meter_loop(self) -> None:
        """Abre um subprocesso Python isolado para medição de nível de áudio.

        O isolamento evita que falhas do PyAudio (driver travado, permissão
        negada, formato incompatível) afetem a UI principal. O subprocesso
        tenta automaticamente diferentes dispositivos, taxas e formatos.
        """
        line_q: queue.Queue[str] = queue.Queue()
        err_q: queue.Queue[str] = queue.Queue()

        def _read_pipe(pipe: Any, target: queue.Queue) -> None:
            try:
                for line in iter(pipe.readline, ""):
                    target.put(line)
            except Exception:
                pass

        proc = None
        try:
            env = dict(os.environ)
            idx = _parse_device_index(self.device)
            env["SYNROV_AUDIO_DEVICE_INDEX"] = "" if idx is None else str(idx)

            proc = subprocess.Popen(
                [sys.executable, "-u", "-c", _METER_RUNNER],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                env=env,
            )
            self._meter_proc = proc

            if proc.stdout:
                threading.Thread(
                    target=_read_pipe, args=(proc.stdout, line_q), daemon=True
                ).start()
            if proc.stderr:
                threading.Thread(
                    target=_read_pipe, args=(proc.stderr, err_q), daemon=True
                ).start()


            started = False
            deadline = time.time() + 6.0
            while self.running and time.time() < deadline:
                if proc.poll() is not None:
                    err = "".join(err_q.queue)[:500]
                    self.audio_backend = "pyaudio_error"
                    self.last_sr_error = (
                        f"Monitor de áudio falhou (código {proc.returncode}): {err}"
                    )
                    self.status_message = self.last_sr_error
                    return
                try:
                    line = line_q.get(timeout=0.10)
                except queue.Empty:
                    continue
                if not line.strip().startswith("{"):
                    continue
                try:
                    msg = json.loads(line.strip())
                except Exception:
                    continue
                if msg.get("ok"):
                    started = True
                    dev_index = msg.get("device_index", idx)
                    dev_name = str(msg.get("device_name", "microfone"))[:120]
                    self.device_label = (
                        f"{dev_index} · {dev_name}"
                        if dev_index not in (None, -1, "")
                        else dev_name
                    )
                    self.audio_backend = (
                        f"pyaudio_{msg.get('format', 'input')}@{msg.get('rate', '?')}"
                    )
                    self.status_message = (
                        f"Monitor de áudio ativo em {self.device_label}."
                    )
                    break

            if not started:
                self.audio_backend = "pyaudio_timeout"
                self.last_sr_error = (
                    "Monitor de áudio não respondeu em 6s. "
                    "Verifique permissão do microfone ou driver."
                )
                self.status_message = self.last_sr_error
                try:
                    proc.terminate()
                except Exception:
                    pass
                return


            while self.running and proc.poll() is None:
                try:
                    line = line_q.get(timeout=0.20)
                except queue.Empty:
                    continue
                if not line.strip().startswith("{"):
                    continue
                try:
                    d = json.loads(line.strip())
                    rms = _clamp(float(d.get("r", 0.0)), 0.0, 1.0)
                    peak = _clamp(float(d.get("p", 0.0)), 0.0, 1.0)
                    pulse = _clamp(float(d.get("u", 0.0)), -1.0, 1.0)

                    nf = _safe_float(self._noise_floor, 0.0035)
                    if rms <= nf * 1.6:
                        nf = 0.985 * nf + 0.015 * max(rms, 0.0001)
                    else:
                        nf = 0.997 * nf + 0.003 * rms
                    self._noise_floor = _clamp(nf, 0.00035, 0.08)

                    visual = _clamp(
                        (max(rms, peak * 0.72) - self._noise_floor * 0.65)
                        / max(0.010, 0.075 - self._noise_floor),
                        0.0, 1.0,
                    )
                    with self._audio_lock:
                        self.audio_meter = _clamp(
                            0.62 * _safe_float(self.audio_meter) + 0.38 * visual,
                            0.0, 1.0,
                        )
                        self.audio_level = rms
                        self.audio_peak = peak


                        self.audio_features = [
                            rms, peak, 0.0, 0.5, 0.33, 0.33, 0.33, pulse
                        ]
                        self.audio_ts = time.time()
                except Exception:
                    continue


            if self.running:
                err = "".join(err_q.queue)[:500]
                self.audio_backend = "pyaudio_stopped"
                if err:
                    self.last_sr_error = f"Monitor de áudio parou: {err}"
                    self.status_message = self.last_sr_error

        except Exception as exc:
            self.audio_backend = "pyaudio_error"
            self.last_sr_error = f"Monitor de áudio: {exc}"
            self.status_message = self.last_sr_error
        finally:
            try:
                if proc is not None and proc.poll() is None:
                    proc.terminate()
            except Exception:
                pass
            self._meter_proc = None


    def _speech_loop(self) -> None:
        """Loop contínuo de escuta e reconhecimento via Google Speech Recognition."""
        if not _SR_AVAILABLE or _sr_lib is None:
            return

        recognizer = _sr_lib.Recognizer()
        try:
            recognizer.operation_timeout = 2.5
            recognizer.pause_threshold = 0.65
            recognizer.non_speaking_duration = 0.38
            recognizer.phrase_threshold = 0.22
            recognizer.energy_threshold = 360
            recognizer.dynamic_energy_threshold = False
        except Exception:
            pass

        selected_idx = _parse_device_index(self.device)
        candidates: List[Optional[int]] = []
        if selected_idx is not None:
            candidates.append(selected_idx)
        candidates.append(None)

        last_open_error = ""
        for mic_index in candidates:
            if not self.running:
                return
            try:
                self.device_label = _safe_device_label(mic_index)
                with _sr_lib.Microphone(device_index=mic_index) as source:
                    self.speech_backend = "google_microphone_ready"
                    self.status_message = (
                        f"Google Speech pronto em {self.device_label}"
                    )
                    try:
                        # reconhecimento com limiar fixo; evita autoajuste que pode mascarar comandos
                        pass
                    except Exception:
                        pass

                    while self.running:
                        try:
                            self.speech_backend = "google_listening"
                            audio = recognizer.listen(
                                source, timeout=0.75, phrase_time_limit=2.6
                            )
                        except _sr_lib.WaitTimeoutError:
                            continue
                        except Exception as exc:
                            self.last_sr_error = f"Falha ao escutar microfone: {exc}"
                            time.sleep(0.25)
                            continue
                        self._recognize_async(recognizer, audio)
                    return

            except Exception as exc:
                last_open_error = str(exc)
                continue


        self.speech_backend = "microphone_error"
        self.last_sr_error = (
            "Falha ao conectar reconhecimento de fala ao microfone. "
            "Barra de som continua medindo o áudio. "
            "Verifique permissões, se outro app usa o microfone e se PyAudio está instalado. "
            f"Detalhe: {last_open_error}"
        )
        self.status_message = self.last_sr_error

    def _recognize_async(self, recognizer: Any, audio: Any) -> None:
        """Envia reconhecimento para thread separada, evitando bloquear o loop de captura."""
        if self._recognition_busy:
            return
        self._recognition_busy = True

        def _worker() -> None:
            text = ""
            last_error = ""
            try:
                for language in self._recognition_languages():
                    if not self.running:
                        return
                    try:
                        self.speech_backend = f"google_recognizing_{language}"
                        text = recognizer.recognize_google(audio, language=language)
                        if text:
                            break
                    except _sr_lib.UnknownValueError:
                        last_error = "não entendi o áudio"
                        text = ""
                    except _sr_lib.RequestError as exc:
                        last_error = (
                            f"Google Speech não respondeu ou sem internet: {exc}"
                        )
                        text = ""
                        break
                    except Exception as exc:
                        last_error = f"Falha no reconhecimento: {exc}"
                        text = ""

                if text:
                    self.inject_text(text)
                    self.speech_backend = "google_recognized"
                    self.status_message = f"Reconhecido: {text}"
                    self.last_sr_error = ""
                elif last_error:
                    self.last_sr_error = last_error
                    self.speech_backend = "google_listening"
            finally:
                self._recognition_busy = False

        self._recognition_thread = threading.Thread(
            target=_worker, name="synrov-google-recognizer", daemon=True
        )
        self._recognition_thread.start()


__all__ = [
    "AUDIO_KEYS",
    "RichVoiceParser",
    "GoogleVoiceAudioEngine",
]
