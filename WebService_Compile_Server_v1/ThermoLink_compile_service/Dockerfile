FROM python:3.11-slim

ENV ARDUINO_FQBN=esp32:esp32:esp32
ENV COMPILE_TIMEOUT_SEC=220

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates xz-utils git bash \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh \
    && mv bin/arduino-cli /usr/local/bin/arduino-cli \
    && arduino-cli version

RUN arduino-cli config init \
    && arduino-cli core update-index \
    && arduino-cli core install esp32:esp32 \
    && arduino-cli lib install WiFiManager \
    && arduino-cli lib install PID

WORKDIR /app
COPY requirements.txt /app/requirements.txt
RUN pip install --no-cache-dir -r /app/requirements.txt
COPY server.py /app/server.py

EXPOSE 5000
CMD ["gunicorn", "-w", "1", "-b", "0.0.0.0:5000", "server:app", "--timeout", "240"]
