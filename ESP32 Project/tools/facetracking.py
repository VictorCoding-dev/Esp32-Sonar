"""Hand tracking with MediaPipe, driving two ESP32 positional servos through Firmata."""

import argparse
import queue
import re
import sys
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import pyfirmata
import serial
from pyfirmata import Board
from pyfirmata.util import Iterator


FIRMATA_PAN_PIN = 9
FIRMATA_TILT_PIN = 10
FIRMATA_LASER_PIN = 26
FIRMATA_DISTANCE_ANALOG_PIN = 0
RADAR_ANGLE_STEP_DEG = 3
RADAR_ANGLE_BIN_COUNT = 181 // RADAR_ANGLE_STEP_DEG + 1
PAN_LEFT_DEG = 170.0
PAN_RIGHT_DEG = 10.0
TILT_TOP_DEG = 35.0
TILT_BOTTOM_DEG = 145.0
SERVO_RESPONSE = 0.9
PALM_LANDMARK_IDS = (0, 5, 9, 13, 17)
ESP32_FIRMATA_LAYOUT = {
    "digital": tuple(range(27)),
    "analog": (0,),
    "pwm": (),
    "disabled": (),
    "use_ports": True,
}
MPU_LINE_PATTERN = re.compile(
    r"Roll:\s*([-+\d.]+)\s*\|\s*Pitch:\s*([-+\d.]+)"
    r"(?:\s*\|\s*Temp:\s*([-+\d.]+)\s*C)?\s*\|\s*Alerta:\s*(SIM|NAO)",
    re.IGNORECASE,
)


def map_hand_to_servos(landmarks, frame_width, frame_height, current_position):
    palm_x = sum(landmarks[index].x for index in PALM_LANDMARK_IDS) / len(PALM_LANDMARK_IDS)
    palm_y = sum(landmarks[index].y for index in PALM_LANDMARK_IDS) / len(PALM_LANDMARK_IDS)
    target_x = float(np.interp(palm_x, [0.0, 1.0], [PAN_LEFT_DEG, PAN_RIGHT_DEG]))
    target_y = float(np.interp(palm_y, [0.0, 1.0], [TILT_TOP_DEG, TILT_BOTTOM_DEG]))
    return [
        current_position[0] + SERVO_RESPONSE * (target_x - current_position[0]),
        current_position[1] + SERVO_RESPONSE * (target_y - current_position[1]),
    ]


def read_mpu_serial(port_name, baud_rate, stop_event, data_queue):
    try:
        mpu_port = serial.Serial()
        mpu_port.port = port_name
        mpu_port.baudrate = baud_rate
        mpu_port.timeout = 0.25
        mpu_port.dtr = False
        mpu_port.rts = False
        mpu_port.open()
    except serial.SerialException as error:
        data_queue.put(("error", str(error)))
        return

    try:
        while not stop_event.is_set():
            line = mpu_port.readline().decode("utf-8", errors="ignore").strip()
            match = MPU_LINE_PATTERN.search(line)
            if match:
                data_queue.put((
                    "data",
                    (float(match.group(1)), float(match.group(2)),
                     float(match.group(3)) if match.group(3) else None,
                     match.group(4).upper() == "SIM", time.monotonic()),
                ))
    except serial.SerialException as error:
        data_queue.put(("error", str(error)))
    finally:
        mpu_port.close()


def draw_mpu_panel(image, mpu_data, mpu_status, distance_cm, target_locked):
    height, width = image.shape[:2]
    panel_width = min(370, width - 30)
    panel_height = 190
    x1 = width - panel_width - 18
    y1 = 18
    x2 = x1 + panel_width
    y2 = y1 + panel_height

    overlay = image.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (16, 30, 38), cv2.FILLED)
    cv2.addWeighted(overlay, 0.88, image, 0.12, 0, image)
    cv2.rectangle(image, (x1, y1), (x2, y2), (78, 145, 145), 1)

    cv2.putText(image, "MPU REMOTO / ESP-NOW", (x1 + 14, y1 + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (180, 255, 210), 2)
    cv2.line(image, (x1 + 14, y1 + 36), (x2 - 14, y1 + 36), (50, 100, 105), 1)

    if mpu_data is None:
        roll_text = "Roll: --"
        pitch_text = "Pitch: --"
        temp_text = "Temp: --"
        mpu_state = mpu_status or "MPU aguardando"
        mpu_color = (170, 185, 185)
    else:
        roll, pitch, temperature, alert, received_at = mpu_data
        fresh = time.monotonic() - received_at < 1.5
        value_color = (100, 230, 205) if fresh else (90, 165, 235)
        roll_text = f"Roll: {roll:5.1f} deg"
        pitch_text = f"Pitch: {pitch:5.1f} deg"
        temp_text = f"Temp: {temperature:.1f} C" if temperature is not None else "Temp: --"
        mpu_state = "MPU ALERTA" if alert else ("MPU ATIVO" if fresh else "MPU SEM DADOS")
        mpu_color = (80, 90, 255) if alert else (value_color if fresh else (70, 170, 245))

    cv2.putText(image, roll_text, (x1 + 14, y1 + 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, mpu_color, 1)
    cv2.putText(image, pitch_text, (x1 + 190, y1 + 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, mpu_color, 1)
    cv2.putText(image, temp_text, (x1 + 14, y1 + 91),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (215, 225, 220), 1)
    distance_text = f"Distancia: {distance_cm} cm" if distance_cm is not None else "Distancia: sem eco"
    cv2.putText(image, distance_text, (x1 + 14, y1 + 119),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 220, 255), 1)

    laser_text = "ALVO LOCKED | LASER ON" if target_locked else "SEM ALVO | LASER OFF"
    laser_color = (80, 90, 255) if target_locked else (100, 230, 150)
    cv2.putText(image, laser_text, (x1 + 14, y1 + 151),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, laser_color, 1)
    cv2.putText(image, mpu_state[:40], (x1 + 14, y1 + 176),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, mpu_color, 1)


def draw_sonar_radar(image, distance_cm, pan_angle, echo_history):
    height, width = image.shape[:2]
    panel_width = min(370, width - 30)
    panel_height = 250
    panel_x = width - panel_width - 18
    panel_y = 220
    panel_right = panel_x + panel_width
    panel_bottom = panel_y + panel_height

    overlay = image.copy()
    cv2.rectangle(overlay, (panel_x, panel_y), (panel_right, panel_bottom), (16, 30, 38), cv2.FILLED)
    cv2.addWeighted(overlay, 0.88, image, 0.12, 0, image)
    cv2.rectangle(image, (panel_x, panel_y), (panel_right, panel_bottom), (78, 145, 145), 1)
    cv2.putText(image, "RADAR ULTRASSONICO", (panel_x + 14, panel_y + 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 210), 2)

    origin = (panel_x + panel_width // 2, panel_bottom - 24)
    radius = min(panel_width // 2 - 26, panel_height - 64)
    grid_color = (55, 105, 110)

    for fraction, label in ((0.25, "100"), (0.5, "200"), (0.75, "300"), (1.0, "400 cm")):
        arc_radius = int(radius * fraction)
        cv2.ellipse(image, origin, (arc_radius, arc_radius), 0, 180, 360, grid_color, 1)
        cv2.putText(image, label, (origin[0] + 5, origin[1] - arc_radius + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (130, 165, 165), 1)

    for angle in range(0, 181, 30):
        radians = np.deg2rad(angle)
        endpoint = (
            int(origin[0] + np.cos(radians) * radius),
            int(origin[1] - np.sin(radians) * radius),
        )
        cv2.line(image, origin, endpoint, grid_color, 1)

    remembered_count = 0
    for bin_index, remembered_distance in enumerate(echo_history):
        if remembered_distance is None:
            continue

        angle = np.deg2rad(bin_index * RADAR_ANGLE_STEP_DEG)
        echo_radius = int(radius * min(remembered_distance, 400) / 400)
        remembered_point = (
            int(origin[0] + np.cos(angle) * echo_radius),
            int(origin[1] - np.sin(angle) * echo_radius),
        )
        cv2.circle(image, remembered_point, 4, (80, 220, 150), cv2.FILLED)
        remembered_count += 1

    cv2.putText(image, f"MEMORIA: {remembered_count} pontos", (panel_x + 14, panel_y + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (100, 220, 160), 1)

    beam_angle = np.deg2rad(np.clip(pan_angle, 0, 180))
    beam_radius = radius if distance_cm is None else int(radius * min(distance_cm, 400) / 400)
    echo_point = (
        int(origin[0] + np.cos(beam_angle) * beam_radius),
        int(origin[1] - np.sin(beam_angle) * beam_radius),
    )
    beam_color = (80, 190, 255) if distance_cm is not None else (70, 125, 130)
    cv2.line(image, origin, echo_point, beam_color, 2)
    if distance_cm is not None:
        cv2.circle(image, echo_point, 6, (0, 220, 255), cv2.FILLED)
        cv2.putText(image, f"ATUAL: {distance_cm} cm", (panel_x + 14, panel_y + 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (100, 220, 255), 1)
    else:
        cv2.putText(image, "ATUAL: SEM ECO", (panel_x + 14, panel_y + 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (150, 175, 175), 1)


def main():
    parser = argparse.ArgumentParser(description="Rastreamento da mao com dois servos ESP32")
    parser.add_argument("--port", default="COM15", help="Porta serial do ESP32")
    parser.add_argument("--camera", type=int, default=0, help="Indice da camera")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--mpu-port", default="COM16", help="Porta serial somente leitura do receptor MPU")
    parser.add_argument("--mpu-baud", type=int, default=115200)
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.camera)
    width, height = 640, 480
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        raise RuntimeError("A camera nao pode ser acessada.")

    board = None
    laser_pin = None
    distance_pin = None
    hand_detector = None
    mpu_stop = threading.Event()
    mpu_queue = queue.Queue()
    mpu_thread = None
    mpu_data = None
    mpu_status = ""
    try:
        board = Board(args.port, layout=ESP32_FIRMATA_LAYOUT, baudrate=args.baud)
        servo_pin_x = board.get_pin(f"d:{FIRMATA_PAN_PIN}:s")
        servo_pin_y = board.get_pin(f"d:{FIRMATA_TILT_PIN}:s")
        laser_pin = board.get_pin(f"d:{FIRMATA_LASER_PIN}:o")
        distance_pin = board.get_pin(f"a:{FIRMATA_DISTANCE_ANALOG_PIN}:i")
        laser_pin.write(False)
        serial_iterator = Iterator(board)
        serial_iterator.start()
        hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )
        servo_position = [90.0, 90.0]
        radar_echo_history = [None] * RADAR_ANGLE_BIN_COUNT
        last_radar_sample_at = 0.0

        if args.mpu_port.upper() == args.port.upper():
            mpu_status = "Use uma porta diferente da serial dos servos"
        else:
            mpu_thread = threading.Thread(
                target=read_mpu_serial,
                args=(args.mpu_port, args.mpu_baud, mpu_stop, mpu_queue),
                daemon=True,
            )
            mpu_thread.start()

        while True:
            success, image = camera.read()
            if not success:
                break

            frame_height, frame_width = image.shape[:2]
            hand_results = hand_detector.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            hands = hand_results.multi_hand_landmarks or []

            if hands:
                hand_landmarks = hands[0]
                mp.solutions.drawing_utils.draw_landmarks(
                    image, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS
                )
                servo_position = map_hand_to_servos(
                    hand_landmarks.landmark, frame_width, frame_height, servo_position
                )
                palm_x = int(np.mean([
                    hand_landmarks.landmark[index].x * frame_width
                    for index in PALM_LANDMARK_IDS
                ]))
                palm_y = int(np.mean([
                    hand_landmarks.landmark[index].y * frame_height
                    for index in PALM_LANDMARK_IDS
                ]))

                cv2.circle(image, (palm_x, palm_y), 45, (0, 220, 100), 2)
                cv2.putText(image, str((palm_x, palm_y)), (palm_x + 15, palm_y - 15),
                            cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
                cv2.line(image, (0, palm_y), (frame_width, palm_y), (0, 0, 0), 2)
                cv2.line(image, (palm_x, frame_height), (palm_x, 0), (0, 0, 0), 2)
                cv2.circle(image, (palm_x, palm_y), 10, (0, 220, 100), cv2.FILLED)
                cv2.putText(image, "HAND LOCKED", (max(10, frame_width - 430), 50),
                            cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            else:
                center = (frame_width // 2, frame_height // 2)
                cv2.putText(image, "NO HAND", (max(10, frame_width - 400), 50),
                            cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
                cv2.circle(image, center, 80, (0, 0, 255), 2)
                cv2.circle(image, center, 15, (0, 0, 255), cv2.FILLED)
                cv2.line(image, (0, center[1]), (frame_width, center[1]), (0, 0, 0), 2)
                cv2.line(image, (center[0], frame_height), (center[0], 0), (0, 0, 0), 2)

            cv2.putText(image, f"Servo X: {int(servo_position[0])} deg", (50, 50),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.putText(image, f"Servo Y: {int(servo_position[1])} deg", (50, 100),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

            try:
                while True:
                    message_type, value = mpu_queue.get_nowait()
                    if message_type == "data":
                        mpu_data = value
                        mpu_status = "Dados recebidos"
                    else:
                        mpu_status = f"Porta MPU indisponivel: {value}"
            except queue.Empty:
                pass

            distance_signal = distance_pin.read()
            if distance_signal is None or distance_signal <= 0:
                distance_cm = None
            else:
                distance_cm = max(0, min(400, round(distance_signal * 1023) - 1))

            now = time.monotonic()
            if now - last_radar_sample_at >= 0.1:
                radar_bin = round(np.clip(servo_position[0], 0, 180) / RADAR_ANGLE_STEP_DEG)
                radar_echo_history[radar_bin] = distance_cm
                last_radar_sample_at = now

            target_locked = bool(hands)
            laser_pin.write(target_locked)
            draw_mpu_panel(image, mpu_data, mpu_status, distance_cm, target_locked)
            draw_sonar_radar(image, distance_cm, servo_position[0], radar_echo_history)

            servo_pin_x.write(servo_position[0])
            servo_pin_y.write(servo_position[1])
            cv2.imshow("Face tracking", image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        mpu_stop.set()
        if laser_pin is not None:
            laser_pin.write(False)
        if board is not None:
            board.exit()
        if hand_detector is not None:
            hand_detector.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, pyfirmata.util.serial.SerialException) as error:
        print(error, file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(0)
