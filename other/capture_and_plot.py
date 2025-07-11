#!/usr/bin/env python3
"""
Captura datos de la ejecución del controlador ON-OFF vía serial durante 5 segundos
y genera gráficas discretas escalonadas:
1. Referencia vs Velocidad (MISO)
2. Referencia vs Error
3. Velocidad vs Señal de Control

Requisitos:
- pyserial  
- matplotlib

Uso:
    python capture_and_plot.py --port COM3
"""
import argparse
import time
import serial
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(description='Captura y grafica datos del controlador ON-OFF')
    parser.add_argument('--port', required=True, help='Puerto serial (ej. COM3 o /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baudrate del puerto serial')
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error abriendo el puerto serial: {e}")
        return

    print(f"Leyendo datos por {args.port} a {args.baud} baudios durante 5 segundos...")
    start = time.time()

    times = []
    refs = []
    speeds = []
    ctrls = []
    errs = []

    # Leer datos durante 5 segundos
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) < 4:
            continue
        try:
            ref = float(parts[0])
            speed = float(parts[1])
            speed = (speed * 255) / 4095
            ctrl = float(parts[2])
            err = float(parts[3])
        except ValueError:
            continue

        t = time.time() - start
        if t > 5.0:
            break

        times.append(t)
        refs.append(ref)
        speeds.append(speed)
        ctrls.append(ctrl)
        errs.append(err)

    ser.close()
    print("Lectura completada. Generando gráficas...")

    # Graficar datos
    plt.figure(figsize=(12, 8))

    # 1. Referencia vs Velocidad
    ax1 = plt.subplot(3, 1, 1)
    ax1.step(times, refs, where='post', label='Referencia')
    ax1.step(times, speeds, where='post', label='Velocidad')
    ax1.set_ylabel('Amplitud')
    ax1.set_title('Referencia vs Velocidad del Motor')
    ax1.legend()

    # 2. Referencia vs Error
    ax2 = plt.subplot(3, 1, 2)
    ax2.step(times, refs, where='post', label='Referencia')
    ax2.step(times, errs, where='post', label='Error')
    ax2.set_ylabel('Amplitud')
    ax2.set_title('Referencia vs Error')
    ax2.legend()

    # 3. Velocidad vs Señal de Control
    ax3 = plt.subplot(3, 1, 3)
    ax3.step(times, speeds, where='post', label='Velocidad')
    ax3.step(times, ctrls, where='post', label='Señal de Control')
    ax3.set_ylabel('Amplitud')
    ax3.set_xlabel('Tiempo [s]')
    ax3.set_title('Velocidad vs Señal de Control')
    ax3.legend()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
