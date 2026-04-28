from ultralytics import YOLO
import cv2
import time
import math
import numpy as np

# Load model
model = YOLO("best.pt")

cap = cv2.VideoCapture(0)
prev_time = 0

# --- KONFIGURASI PARAMETER ---
BOX_SIZE = 200  # Ukuran kotak tengah (Deadzone)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    # Koordinat Deadzone
    x_min, y_min = cx - (BOX_SIZE // 2), cy - (BOX_SIZE // 2)
    x_max, y_max = cx + (BOX_SIZE // 2), cy + (BOX_SIZE // 2)

    # Run YOLO detection
    results = model(frame)
    
    # List untuk menampung data array float
    raw_data = []

    # Gambar Kotak Parameter Tengah (Deadzone)
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 255, 255), 2)
    cv2.putText(frame, "DEADZONE", (x_min, y_min - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    for r in results:
        boxes = r.boxes
        if boxes is not None:
            for box in boxes:
                # Koordinat Bounding Box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Info Class & Confidence
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls_id]

                # Titik Tengah Objek
                tx, ty = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Jarak Euclidean
                dist = math.sqrt((tx - cx)**2 + (ty - cy)**2)

                # --- LOGIKA NAVIGASI ---
                # 0.0: Tengah, 1.0: Kiri, 2.0: Kanan
                if tx < x_min:
                    command_str = "PUTAR KIRI"
                    cmd_code = 1.0
                    color = (0, 0, 255) # Merah
                elif tx > x_max:
                    command_str = "PUTAR KANAN"
                    cmd_code = 2.0
                    color = (0, 0, 255) # Merah
                else:
                    command_str = "POSISI AMAN"
                    cmd_code = 0.0
                    color = (0, 255, 0) # Hijau

                # --- SIMPAN DATA KE LIST (FLOAT) ---
                # Format: [ClassID, Conf, Dist, CentX, CentY, CmdCode]
                raw_data.append([float(cls_id), round(conf, 2), round(dist, 2), 
                                 float(tx), float(ty), cmd_code])

                # --- VISUALISASI PADA FRAME ---
                # 1. Bounding Box & Titik Tengah
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.circle(frame, (tx, ty), 5, color, -1)
                
                # 2. Garis ke Pusat Layar
                cv2.line(frame, (cx, cy), (tx, ty), (255, 255, 0), 1)

                # 3. Label Keterangan (Atas Box)
                info_text = f"{label} {conf:.2f} | Dist: {int(dist)}px"
                cv2.putText(frame, info_text, (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                # 4. Keterangan Perintah (Bawah Box)
                cv2.putText(frame, command_str, (x1, y2 + 25), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # --- OUTPUT TERMINAL: ARRAY FLOAT ---
    if raw_data:
        np_array = np.array(raw_data, dtype=np.float32)
        print("Data Array Float:")
        print(np_array)
    else:
        print("Searching...")

    # Hitung FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
    prev_time = curr_time
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    # Tampilkan Frame
    cv2.imshow("YOLO Tracker - Float Array Output", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

