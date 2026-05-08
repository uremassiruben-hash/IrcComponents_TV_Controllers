import os
import subprocess
import tempfile
from pathlib import Path
from flask import Flask, request, send_file, jsonify

app = Flask(__name__)

FQBN = os.environ.get("ARDUINO_FQBN", "esp32:esp32:esp32")
API_TOKEN = os.environ.get("THERMOLINK_COMPILE_TOKEN", "").strip()
COMPILE_TIMEOUT_SEC = int(os.environ.get("COMPILE_TIMEOUT_SEC", "220"))
MAX_CODE_BYTES = int(os.environ.get("MAX_CODE_BYTES", "600000"))


def _authorized() -> bool:
    if not API_TOKEN:
        return True
    supplied = request.headers.get("X-Compile-Token", "") or request.form.get("token", "")
    return supplied == API_TOKEN


@app.get("/health")
def health():
    return jsonify(
        ok=True,
        service="ThermoLink Arduino ESP32 Compile Service",
        fqbn=FQBN,
        auth_enabled=bool(API_TOKEN),
    )


@app.post("/compile")
def compile_ino():
    if not _authorized():
        return jsonify(error="Token server compilazione non valido"), 401

    code = request.form.get("code", "")
    if not code and request.is_json:
        code = (request.get_json(silent=True) or {}).get("code", "")
    if not code and "file" in request.files:
        code = request.files["file"].read().decode("utf-8", errors="replace")

    if not code.strip():
        return jsonify(error="Codice .ino mancante. Invia il campo form 'code'."), 400
    if len(code.encode("utf-8")) > MAX_CODE_BYTES:
        return jsonify(error=f"Codice troppo grande. Max {MAX_CODE_BYTES} byte."), 413
    if "setup" not in code or "loop" not in code:
        return jsonify(error="Codice .ino incompleto: devono esistere setup() e loop()."), 400

    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        sketch_dir = tmpdir / "ThermoLinkFirmware"
        out_dir = tmpdir / "out"
        sketch_dir.mkdir(parents=True, exist_ok=True)
        out_dir.mkdir(parents=True, exist_ok=True)

        ino_path = sketch_dir / "ThermoLinkFirmware.ino"
        ino_path.write_text(code, encoding="utf-8")

        cmd = [
            "arduino-cli", "compile",
            "--fqbn", FQBN,
            "--export-binaries",
            "--output-dir", str(out_dir),
            str(sketch_dir),
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=COMPILE_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired:
            return jsonify(error=f"Compilazione scaduta dopo {COMPILE_TIMEOUT_SEC} secondi"), 504

        if result.returncode != 0:
            return jsonify(
                error="Compilazione fallita",
                command=" ".join(cmd),
                stdout=result.stdout[-6000:],
                stderr=result.stderr[-6000:],
            ), 400

        bins = list(out_dir.rglob("*.bin"))
        app_bins = [
            p for p in bins
            if "bootloader" not in p.name.lower()
            and "partitions" not in p.name.lower()
            and "merged" not in p.name.lower()
        ]
        if not app_bins:
            return jsonify(
                error="Nessun .bin applicativo trovato dopo la compilazione",
                files=[p.name for p in bins],
                stdout=result.stdout[-6000:],
            ), 500

        firmware_bin = max(app_bins, key=lambda p: p.stat().st_size)
        response = send_file(
            firmware_bin,
            mimetype="application/octet-stream",
            as_attachment=True,
            download_name="firmware.bin",
        )
        response.headers["X-Firmware-Size"] = str(firmware_bin.stat().st_size)
        return response


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
