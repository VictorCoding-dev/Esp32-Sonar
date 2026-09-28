"""Track a selected camera object and drive the ESP32 continuous-rotation servos."""

import argparse
import sys
import time

import cv2
import serial


def create_tracker():
    constructors = (
        lambda: cv2.TrackerCSRT_create(),
        lambda: cv2.legacy.TrackerCSRT_create(),
        lambda: cv2.TrackerKCF_create(),
        lambda: cv2.legacy.TrackerKCF_create(),
    )
    for constructor in constructors:
        try:
            return constructor()
        except (AttributeError, cv2.error):
            continue
    raise RuntimeError("Esta instalacao do OpenCV nao possui um tracker CSRT ou KCF.")


def send_position(porta, centro_x, centro_y, largura, altura):
    x_normalizado = int(max(0, min(1000, centro_x * 1000 / largura)))
    y_normalizado = int(max(0, min(1000, centro_y * 1000 / altura)))
    porta.write(f"TRACK,{x_normalizado},{y_normalizado}\n".encode("ascii"))


def main():
    parser = argparse.ArgumentParser(description="Rastreamento de objeto para servos do ESP32")
    parser.add_argument("--port", required=True, help="Porta serial do ESP32, por exemplo COM15")
    parser.add_argument("--camera", type=int, default=0, help="Indice da camera do laptop")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    camera = cv2.VideoCapture(args.camera)
    if not camera.isOpened():
        raise RuntimeError(f"Nao foi possivel abrir a camera {args.camera}.")

    try:
        porta = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as erro:
        camera.release()
        raise RuntimeError(f"Nao foi possivel abrir {args.port}: {erro}") from erro

    try:
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError("Nao foi possivel capturar o primeiro frame da camera.")

        roi = cv2.selectROI("Selecione o objeto e pressione ENTER", frame, False, False)
        cv2.destroyWindow("Selecione o objeto e pressione ENTER")
        if roi[2] <= 0 or roi[3] <= 0:
            raise RuntimeError("Nenhuma area foi selecionada.")

        tracker = create_tracker()
        tracker.init(frame, roi)
        ultima_transmissao = 0.0

        while True:
            ok, frame = camera.read()
            if not ok:
                break

            rastreado, caixa = tracker.update(frame)
            if rastreado:
                x, y, largura, altura = [int(valor) for valor in caixa]
                centro_x = x + largura // 2
                centro_y = y + altura // 2
                agora = time.monotonic()
                if agora - ultima_transmissao >= 0.04:
                    send_position(porta, centro_x, centro_y, frame.shape[1], frame.shape[0])
                    ultima_transmissao = agora
                cv2.rectangle(frame, (x, y), (x + largura, y + altura), (0, 220, 100), 2)
                cv2.circle(frame, (centro_x, centro_y), 5, (0, 220, 100), -1)
                cv2.putText(frame, "RASTREAMENTO", (x, max(25, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 100), 2)
            else:
                cv2.putText(frame, "OBJETO PERDIDO - pressione Q", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 70, 255), 2)

            cv2.imshow("Rastreamento ESP32 - Q para sair", frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        porta.close()
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, KeyboardInterrupt) as erro:
        print(erro, file=sys.stderr)
        sys.exit(1)