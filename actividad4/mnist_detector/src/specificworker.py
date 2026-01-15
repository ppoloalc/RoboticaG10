#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from matplotlib.bezier import inside_circle
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import numpy as np
import time
import traceback
import cv2
import torch
import itertools
import math
import os
import torch.nn as nn
import torch.nn.functional as F

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        # 1st Conv Layer: 1 input channel (grayscale), 32 output channels, 3x3 kernel
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, padding=1)
        # 2nd Conv Layer: 32 input channels, 64 output channels, 3x3 kernel
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, padding=1)
        # Pooling layer
        self.pool = nn.MaxPool2d(2, 2)
        # Fully Connected Layers
        # Image reduces to 7x7 after two poolings (28->14->7)
        self.fc1 = nn.Linear(64 * 7 * 7, 128)
        self.fc2 = nn.Linear(128, 10) # Output layer (10 digits)

    def forward(self, x):
        # Conv1 -> Relu -> MaxPool
        x = self.pool(F.relu(self.conv1(x)))
        # Conv2 -> Relu -> MaxPool
        x = self.pool(F.relu(self.conv2(x)))
        # Flatten
        x = x.view(-1, 64 * 7 * 7)
        # FC Layers
        x = F.relu(self.fc1(x))
        x = self.fc2(x)
        return x

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        # 1. Inicializamos la variable por seguridad
        self.model = None
        # Cargar el modelo
        try:
            # map_location='cpu' es importante si entrenaste en GPU y ejecutas en CPU
            self.model = torch.load("/home/usuario/RoboticaG10/actividad4/my_network.pt", map_location=torch.device('cpu'), weights_only=False)
            self.model.eval()
            print("Model loaded successfully")
        except Exception as e:
            print(f"Error loading model: {e}")


        if startup_check:
            self.startup_check()
        else:
            started_camera = False
            while not started_camera:
                try:
                    print("Connecting to Camera360RGB...")
                    self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                    print("Camera specs:")
                    print(" width:", self.rgb_original.width)
                    print(" height:", self.rgb_original.height)
                    print(" focalx", self.rgb_original.focalx)
                    print(" focaly", self.rgb_original.focaly)
                    print(" period", self.rgb_original.period)
                    print(" ratio {:.2f}".format(self.rgb_original.width / self.rgb_original.height))
                    started_camera = True
                    print("Connected to Camera360RGB")
                except Ice.Exception as e:
                    traceback.print_exc()
                    print(e, "Trying again CAMERA...")

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    @QtCore.Slot()
    def compute(self):

        image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
        color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
        rect = self.detect_frame(color)
        color_copy = color.copy()
        if rect is not None:
            x1, y1, x2, y2 = rect
            cv2.rectangle(color_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            roi = color[y1:y2, x1:x2]
            #cv2.imshow("Detected ROI", roi)
        cv2.imshow("Camera360RGB", color_copy)
        cv2.waitKey(1)

    ################################################################

    def detect_frame(self, color):
        color_copy = color.copy()
        h_img, w_img = color.shape[:2]

        # Convertir a HSV
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

        # Definir rangos de rojo (hay dos rangos en HSV)
        # Rangos de rojo puro
        lower_red1 = np.array([0, 150, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 100])
        upper_red2 = np.array([180, 255, 255])


        # Crear máscara combinando ambos rangos
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Suavizar la máscara para eliminar ruido
        mask = cv2.medianBlur(mask, 5)

        # Encontrar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("No red contours found")
            return None

        best_cnt = None
        best_score = -1
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 0.001 * w_img * h_img:  # descartar muy pequeños
                continue

            # Aproximar contorno a polígono
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)

            # Bounding box
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect_ratio = bw / float(bh)

            # Comprobar "casi cuadrado"
            if 0.5 <= aspect_ratio <= 2.0:
                candidates.append((area, x, y, bw, bh))

        # Score basado en cantidad de pixeles rojos en la máscara
        for c in candidates:
            _, x, y, bw, bh = c
            roi = mask[y:y+bh, x:x+bw]
            red_pixels = cv2.countNonZero(roi)
            total_pixels = bw * bh
            red_ratio = red_pixels / total_pixels
            score = red_ratio
            if score > best_score:
                best_score = score
                best_cnt = c

        if best_cnt is not None:
            _, x, y, bw, bh = best_cnt
            margin = int(min(bw, bh) * 5 / 100)  # 5% margen
            x1 = max(0, x + margin)
            y1 = max(0, y + margin)
            x2 = min(w_img, x + bw - margin)
            y2 = min(h_img, y + bh - margin)

            if x2 <= x1 or y2 <= y1:
                return None

            return [x1, y1, x2, y2]
        else:
            return None

    ####################################################################
    def startup_check(self):
        print(f"Testing RoboCompCamera360RGB.TRoi from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TRoi()
        print(f"Testing RoboCompCamera360RGB.TImage from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TImage()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods the component implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getNumber method from MNIST interface
    #
    def MNIST_getNumber(self):

        # 0. Verificaciones previas
        if self.model is None:
            print("Model not loaded.")
            return -1

        try:
            # 1. Obtener imagen actual desde el proxy
            # Usamos getROI(-1...) para obtener la imagen completa
            image_data = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
            color = np.frombuffer(image_data.image, dtype=np.uint8).reshape(image_data.height, image_data.width, 3)

            # 2. Detectar región candidata (ROI)
            rect = self.detect_frame(color)

            if rect is None:
                # No se encontró ningún candidato a número
                return -1

            x1, y1, x2, y2 = rect
            roi = color[y1:y2, x1:x2]

            centro_x = (x1 + x2) // 2

            # 3. Preprocesamiento para la Red Neuronal
            # Convertir a escala de grises
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # Redimensionar a 28x28 (tamaño estándar MNIST)
            resized = cv2.resize(gray_roi, (28, 28), interpolation=cv2.INTER_AREA)

            # Normalizar (0-1) y convertir a Tensor
            # La entrada debe ser Float32
            img_tensor = torch.from_numpy(resized).float() / 255.0

            # Añadir dimensiones para batch y canales: [1, 1, 28, 28]
            img_tensor = img_tensor.unsqueeze(0).unsqueeze(0)

            # 4. Inferencia (Forward pass)
            with torch.no_grad():
                predictions = self.model(img_tensor)

                # Obtener el índice con mayor probabilidad
                predicted_digit = torch.argmax(predictions, dim=1).item()

                # Opcional: Imprimir confianza para depurar
                # probs = torch.nn.functional.softmax(predictions, dim=1)
                # print(f"Detected: {predicted_digit} with conf: {probs.max().item():.2f}")

            res = ifaces.RoboCompMNIST.MNistResult(label = predicted_digit, centrox = centro_x)

            return res

        except Exception as e:
            print(f"Error in MNIST_getNumber: {e}")
            res = ifaces.RoboCompMNIST.MNistResult(label = -1, centrox = -1)
            #res.label = -1
            #res.centrox = -1
            return res

        # call DNN and return detection result


    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCamera360RGB you can call this methods:
    # RoboCompCamera360RGB.TImage self.camera360rgb_proxy.getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

    ######################
    # From the RoboCompCamera360RGB you can use this types:
    # ifaces.RoboCompCamera360RGB.TRoi
    # ifaces.RoboCompCamera360RGB.TImage


