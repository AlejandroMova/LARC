# --- INSTALACIÓN ---
# pip install opencv-python-headless numpy pyserial

import cv2
import numpy as np
import serial
import time
import subprocess # Para llamar a comandos del sistema (rp-cam)
import os         # Para manejar archivos temporales

# --- 1. CONFIGURACIÓN ---

# Conexión Serial al Arduino
try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(2) 
    print("Conexión serial con Arduino establecida.")
except serial.SerialException as e:
    print(f"Error al conectar con Arduino: {e}")
    ser = None

# Rango de color (Naranja) en HSV. ¡Este es el ajuste principal!
lower_orange = np.array([5, 100, 100])
upper_orange = np.array([20, 255, 255])

# Archivo temporal para la captura
temp_image_file = "capture.jpg"

print("Iniciando bucle de detección. Presiona 'q' en la ventana para salir.")

# --- 2. BUCLE PRINCIPAL ---

try:
    while True:
        # --- Captura de Cámara (usando rp-cam) ---
        # Llama al comando de terminal para tomar una foto
        capture_command = [
            "rp-cam",
            "--lores", "640", "480", # Resolución
            "-t", "1",
            "--encoding", "jpg",
            "--nopreview",
            "-o", temp_image_file
        ]

        try:
            # Ejecuta el comando
            subprocess.run(capture_command, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print(f"Falló 'rp-cam': {e}. Reintentando...")
            time.sleep(1)
            continue # Salta esta iteración

        # Lee la foto que acabamos de guardar
        frame = cv2.imread(temp_image_file)

        if frame is None:
            print(f"No pude leer la foto {temp_image_file}")
            time.sleep(1)
            continue
        
        # --- Procesamiento de Imagen ---
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Crear y limpiar la máscara de color
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Detección de Círculos
        # Ajustar 'param2', 'minRadius' y 'maxRadius' es clave aquí.
        circles = cv2.HoughCircles(
            mask,
            cv2.HOUGH_GRADIENT,
            dp=1.2,          
            minDist=100,     # Distancia mínima entre círculos
            param1=100,      
            param2=20,       # Umbral de "confianza" (más bajo = detecta más)
            minRadius=10,    # Radio mínimo
            maxRadius=200    # Radio máximo
        )

        # --- Envío de Datos ---
        if circles is not None:
            # Si encontramos un círculo, tomamos el primero
            circles = np.uint16(np.around(circles))
            i = circles[0][0]
            x, y, r = i[0], i[1], i[2]
            
            # Dibujar (para depuración)
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            
            # Formato de datos: <x,y,r>
            data_str = f"<{x},{y},{r}>\n"
            
            # Enviar al Arduino si está conectado
            if ser:
                try:
                    ser.write(data_str.encode('utf-8'))
                    print(f"Enviado: {data_str.strip()}")
                except serial.SerialException as e:
                    print(f"Error al enviar datos: {e}")

        # --- Mostrar Ventanas y Salir ---
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        # Presiona 'q' para salir
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("Deteniendo script...")
            break

finally:
    # --- 3. LIMPIEZA ---
    print("Limpiando recursos...")
    if ser and ser.is_open:
        ser.close() # Cierra conexión serial
    cv2.destroyAllWindows() # Cierra ventanas
    
    # Borra la foto temporal
    if os.path.exists(temp_image_file):
        os.remove(temp_image_file)
        
    print("Script finalizado.")