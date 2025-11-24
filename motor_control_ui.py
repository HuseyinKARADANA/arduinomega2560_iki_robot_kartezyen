#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arduino Step Motor Kontrol Arayüzü
PyQt6 ile geliştirilmiş motor kontrol paneli
"""

import sys
import serial
import serial.tools.list_ports
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QLineEdit, 
                             QComboBox, QGroupBox, QGridLayout, QMessageBox,
                             QStatusBar, QTextEdit, QFileDialog, QProgressBar,
                             QSplitter, QScrollArea)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QFont, QPalette, QColor


class SerialReaderThread(QThread):
    """Seri porttan gelen verileri okuyan thread"""
    data_received = pyqtSignal(str)
    
    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True
    
    def run(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_received.emit(line)
            except Exception as e:
                print(f"Seri port okuma hatası: {e}")
                break
    
    def stop(self):
        self.running = False


class GCodeRunnerThread(QThread):
    """G-code komutlarını çalıştıran thread"""
    progress = pyqtSignal(int, int)  # current, total
    finished = pyqtSignal()
    error = pyqtSignal(str)
    
    def __init__(self, serial_connection, gcode_lines):
        super().__init__()
        self.serial_connection = serial_connection
        self.gcode_lines = gcode_lines
        self.running = True
        self.paused = False
    
    def run(self):
        try:
            total_lines = len(self.gcode_lines)
            for i, line in enumerate(self.gcode_lines):
                if not self.running:
                    break
                
                # Pause kontrolü
                while self.paused and self.running:
                    self.msleep(100)
                
                if not self.running:
                    break
                
                line = line.strip()
                # Boş satırları ve yorumları atla
                if not line or line.startswith(';') or line.startswith('('):
                    self.progress.emit(i + 1, total_lines)
                    continue
                
                # G-code komutunu gönder
                try:
                    self.serial_connection.write(f"{line}\n".encode('utf-8'))
                    # Komutun işlenmesi için kısa bir bekleme
                    self.msleep(50)
                    self.progress.emit(i + 1, total_lines)
                except Exception as e:
                    self.error.emit(f"Satır {i+1} gönderilemedi: {str(e)}")
                    break
            
            self.finished.emit()
        except Exception as e:
            self.error.emit(f"G-code çalıştırma hatası: {str(e)}")
    
    def stop(self):
        self.running = False
    
    def pause(self):
        self.paused = True
    
    def resume(self):
        self.paused = False


class MotorControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_connection = None
        self.reader_thread = None
        self.gcode_runner = None
        self.current_axis = 'X'
        self.gcode_lines = []
        self.init_ui()
        self.update_port_list()
        
        # Port listesini periyodik olarak güncelle
        self.port_timer = QTimer()
        self.port_timer.timeout.connect(self.update_port_list)
        self.port_timer.start(2000)  # 2 saniyede bir
    
    def init_ui(self):
        self.setWindowTitle("Arduino Step Motor Kontrol Paneli")
        self.setGeometry(100, 100, 1200, 800)
        
        # Scroll area oluştur
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        
        # Ana widget (scroll area içine konulacak)
        main_widget = QWidget()
        main_layout = QVBoxLayout()
        main_widget.setLayout(main_layout)
        
        # Scroll area'ya widget'ı ekle
        scroll_area.setWidget(main_widget)
        
        # Central widget olarak scroll area'yı ayarla
        self.setCentralWidget(scroll_area)
        
        # Bağlantı paneli
        connection_group = QGroupBox("Seri Port Bağlantısı")
        connection_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.refresh_btn = QPushButton("Yenile")
        self.refresh_btn.clicked.connect(self.update_port_list)
        
        self.connect_btn = QPushButton("Bağlan")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
        
        self.disconnect_btn = QPushButton("Bağlantıyı Kes")
        self.disconnect_btn.clicked.connect(self.toggle_connection)
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold; padding: 5px;")
        
        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.refresh_btn)
        connection_layout.addWidget(self.connect_btn)
        connection_layout.addWidget(self.disconnect_btn)
        connection_layout.addStretch()
        
        connection_group.setLayout(connection_layout)
        main_layout.addWidget(connection_group)
        
        # Motor kontrolleri - Grid layout
        motors_group = QGroupBox("Motor Kontrolleri")
        motors_layout = QGridLayout()
        
        # Motor isimleri ve kontrolleri
        self.motors = ['X', 'Y', 'Z', 'E', 'R', 'T']
        self.motor_widgets = {}
        
        row = 0
        for motor in self.motors:
            # Motor adı
            motor_label = QLabel(f"Motor {motor}:")
            motor_label.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            motors_layout.addWidget(motor_label, row, 0)
            
            # Hız input
            speed_label = QLabel("Hız (us/step):")
            speed_input = QLineEdit()
            speed_input.setPlaceholderText("örn: 620")
            speed_input.setMaximumWidth(100)
            if motor == 'X':
                speed_input.setText("620")
            elif motor == 'Y':
                speed_input.setText("300")
            elif motor == 'Z':
                speed_input.setText("50")
            else:
                speed_input.setText("1500")
            
            speed_layout = QHBoxLayout()
            speed_layout.addWidget(speed_label)
            speed_layout.addWidget(speed_input)
            speed_widget = QWidget()
            speed_widget.setLayout(speed_layout)
            motors_layout.addWidget(speed_widget, row, 1)
            
            # Step sayısı input
            step_label = QLabel("Step Sayısı:")
            step_input = QLineEdit()
            step_input.setPlaceholderText("örn: 1000")
            step_input.setMaximumWidth(100)
            step_input.setText("0")
            
            step_layout = QHBoxLayout()
            step_layout.addWidget(step_label)
            step_layout.addWidget(step_input)
            step_widget = QWidget()
            step_widget.setLayout(step_layout)
            motors_layout.addWidget(step_widget, row, 2)
            
            # Butonlar
            forward_btn = QPushButton("İleri")
            forward_btn.setStyleSheet("background-color: #2196F3; color: white; padding: 5px;")
            forward_btn.clicked.connect(lambda checked, m=motor: self.move_forward(m))
            
            backward_btn = QPushButton("Geri")
            backward_btn.setStyleSheet("background-color: #FF9800; color: white; padding: 5px;")
            backward_btn.clicked.connect(lambda checked, m=motor: self.move_backward(m))
            
            stop_btn = QPushButton("Dur")
            stop_btn.setStyleSheet("background-color: #f44336; color: white; padding: 5px;")
            stop_btn.clicked.connect(lambda checked, m=motor: self.stop_motor(m))
            
            btn_layout = QHBoxLayout()
            btn_layout.addWidget(forward_btn)
            btn_layout.addWidget(backward_btn)
            btn_layout.addWidget(stop_btn)
            btn_widget = QWidget()
            btn_widget.setLayout(btn_layout)
            motors_layout.addWidget(btn_widget, row, 3)
            
            # Widget'ları sakla
            self.motor_widgets[motor] = {
                'speed': speed_input,
                'steps': step_input,
                'forward': forward_btn,
                'backward': backward_btn,
                'stop': stop_btn
            }
            
            row += 1
        
        motors_group.setLayout(motors_layout)
        main_layout.addWidget(motors_group)
        
        # Servo kontrolü
        servo_group = QGroupBox("Servo Kontrolü")
        servo_layout = QHBoxLayout()
        
        servo_layout.addWidget(QLabel("Açı (0-180):"))
        self.servo_angle_input = QLineEdit()
        self.servo_angle_input.setPlaceholderText("örn: 90")
        self.servo_angle_input.setText("90")
        self.servo_angle_input.setMaximumWidth(100)
        
        self.servo_set_btn = QPushButton("Açıyı Ayarla")
        self.servo_set_btn.setStyleSheet("background-color: #9C27B0; color: white; padding: 5px;")
        self.servo_set_btn.clicked.connect(self.set_servo_angle)
        
        servo_layout.addWidget(self.servo_angle_input)
        servo_layout.addWidget(self.servo_set_btn)
        servo_layout.addStretch()
        
        servo_group.setLayout(servo_layout)
        main_layout.addWidget(servo_group)
        
        # G-code paneli
        gcode_group = QGroupBox("G-code Kontrolü")
        gcode_layout = QVBoxLayout()
        
        # G-code butonları
        gcode_btn_layout = QHBoxLayout()
        
        self.load_gcode_btn = QPushButton("G-code Dosyası Yükle")
        self.load_gcode_btn.setStyleSheet("background-color: #607D8B; color: white; padding: 5px;")
        self.load_gcode_btn.clicked.connect(self.load_gcode_file)
        
        self.run_gcode_btn = QPushButton("G-code Çalıştır")
        self.run_gcode_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 5px;")
        self.run_gcode_btn.clicked.connect(self.run_gcode)
        self.run_gcode_btn.setEnabled(False)
        
        self.pause_gcode_btn = QPushButton("Duraklat")
        self.pause_gcode_btn.setStyleSheet("background-color: #FF9800; color: white; padding: 5px;")
        self.pause_gcode_btn.clicked.connect(self.pause_gcode)
        self.pause_gcode_btn.setEnabled(False)
        
        self.stop_gcode_btn = QPushButton("Durdur")
        self.stop_gcode_btn.setStyleSheet("background-color: #f44336; color: white; padding: 5px;")
        self.stop_gcode_btn.clicked.connect(self.stop_gcode)
        self.stop_gcode_btn.setEnabled(False)
        
        gcode_btn_layout.addWidget(self.load_gcode_btn)
        gcode_btn_layout.addWidget(self.run_gcode_btn)
        gcode_btn_layout.addWidget(self.pause_gcode_btn)
        gcode_btn_layout.addWidget(self.stop_gcode_btn)
        gcode_btn_layout.addStretch()
        
        # G-code editörü
        self.gcode_editor = QTextEdit()
        self.gcode_editor.setFont(QFont("Consolas", 9))
        self.gcode_editor.setPlaceholderText("G-code komutlarını buraya yapıştırın veya dosya yükleyin...\n\nÖrnek:\nG28\nG1 X10 Y20 F1000\nG1 X0 Y0 F1000")
        
        # İlerleme çubuğu
        self.gcode_progress = QProgressBar()
        self.gcode_progress.setMinimum(0)
        self.gcode_progress.setMaximum(100)
        self.gcode_progress.setValue(0)
        
        gcode_layout.addLayout(gcode_btn_layout)
        gcode_layout.addWidget(self.gcode_editor)
        gcode_layout.addWidget(QLabel("İlerleme:"))
        gcode_layout.addWidget(self.gcode_progress)
        
        gcode_group.setLayout(gcode_layout)
        main_layout.addWidget(gcode_group)
        
        # Seri port çıktısı
        output_group = QGroupBox("Arduino Çıktısı")
        output_layout = QVBoxLayout()
        
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setMaximumHeight(150)
        self.output_text.setFont(QFont("Consolas", 9))
        
        clear_btn = QPushButton("Temizle")
        clear_btn.clicked.connect(self.output_text.clear)
        
        output_layout.addWidget(self.output_text)
        output_layout.addWidget(clear_btn)
        
        output_group.setLayout(output_layout)
        main_layout.addWidget(output_group)
        
        # Durum çubuğu
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Bağlantı bekleniyor...")
    
    def update_port_list(self):
        """Mevcut seri portları listele"""
        current_selection = self.port_combo.currentText()
        self.port_combo.clear()
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
        
        # Önceki seçimi geri yükle
        index = self.port_combo.findText(current_selection)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)
    
    def toggle_connection(self):
        """Seri port bağlantısını aç/kapat"""
        if self.serial_connection is None or not self.serial_connection.is_open:
            self.connect_to_arduino()
        else:
            self.disconnect_from_arduino()
    
    def connect_to_arduino(self):
        """Arduino'ya bağlan"""
        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "Hata", "Hiç seri port bulunamadı!")
            return
        
        port_name = self.port_combo.currentData()
        if not port_name:
            QMessageBox.warning(self, "Hata", "Lütfen bir port seçin!")
            return
        
        try:
            self.serial_connection = serial.Serial(
                port=port_name,
                baudrate=115200,
                timeout=1,
                write_timeout=1
            )
            
            # Reader thread'i başlat
            self.reader_thread = SerialReaderThread(self.serial_connection)
            self.reader_thread.data_received.connect(self.on_serial_data)
            self.reader_thread.start()
            
            # UI güncelle
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.port_combo.setEnabled(False)
            self.statusBar.showMessage(f"Bağlı: {port_name} @ 115200 baud")
            
            QMessageBox.information(self, "Başarılı", f"{port_name} portuna bağlanıldı!")
            
        except Exception as e:
            QMessageBox.critical(self, "Bağlantı Hatası", f"Bağlantı kurulamadı:\n{str(e)}")
            if self.serial_connection:
                self.serial_connection.close()
                self.serial_connection = None
    
    def disconnect_from_arduino(self):
        """Arduino bağlantısını kes"""
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread.wait()
            self.reader_thread = None
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        
        self.serial_connection = None
        
        # UI güncelle
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.statusBar.showMessage("Bağlantı kesildi")
    
    def send_command(self, command):
        """Arduino'ya komut gönder"""
        if not self.serial_connection or not self.serial_connection.is_open:
            QMessageBox.warning(self, "Hata", "Arduino'ya bağlı değilsiniz!")
            return False
        
        try:
            self.serial_connection.write(f"{command}\n".encode('utf-8'))
            self.output_text.append(f">>> {command}")
            return True
        except Exception as e:
            QMessageBox.critical(self, "Gönderme Hatası", f"Komut gönderilemedi:\n{str(e)}")
            return False
    
    def on_serial_data(self, data):
        """Seri porttan gelen veriyi işle"""
        self.output_text.append(f"<<< {data}")
        # Scroll to bottom
        scrollbar = self.output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def move_forward(self, motor):
        """Motoru ileri hareket ettir"""
        if not self.check_connection():
            return
        
        # Önce ekseni seç
        self.send_command(motor.lower())
        
        # Hızı ayarla
        speed = self.motor_widgets[motor]['speed'].text()
        if speed:
            try:
                speed_val = int(speed)
                self.send_command(f"v={speed_val}")
            except ValueError:
                QMessageBox.warning(self, "Hata", "Geçersiz hız değeri!")
                return
        
        # Step sayısını kontrol et
        steps = self.motor_widgets[motor]['steps'].text()
        if steps:
            try:
                step_val = int(steps)
                if step_val > 0:
                    # Belirli step sayısı kadar hareket
                    self.send_command(f"m={step_val}")
                else:
                    # Sınırsız hareket
                    self.send_command("a")
            except ValueError:
                # Sınırsız hareket
                self.send_command("a")
        else:
            # Sınırsız hareket
            self.send_command("a")
    
    def move_backward(self, motor):
        """Motoru geri hareket ettir"""
        if not self.check_connection():
            return
        
        # Önce ekseni seç
        self.send_command(motor.lower())
        
        # Hızı ayarla
        speed = self.motor_widgets[motor]['speed'].text()
        if speed:
            try:
                speed_val = int(speed)
                self.send_command(f"v={speed_val}")
            except ValueError:
                QMessageBox.warning(self, "Hata", "Geçersiz hız değeri!")
                return
        
        # Step sayısını kontrol et
        steps = self.motor_widgets[motor]['steps'].text()
        if steps:
            try:
                step_val = int(steps)
                if step_val > 0:
                    # Belirli step sayısı kadar hareket
                    self.send_command(f"n={step_val}")
                else:
                    # Sınırsız hareket
                    self.send_command("d")
            except ValueError:
                # Sınırsız hareket
                self.send_command("d")
        else:
            # Sınırsız hareket
            self.send_command("d")
    
    def stop_motor(self, motor):
        """Motoru durdur"""
        if not self.check_connection():
            return
        
        # Önce ekseni seç
        self.send_command(motor.lower())
        # Dur komutu gönder
        self.send_command("w")
    
    def set_servo_angle(self):
        """Servo açısını ayarla"""
        if not self.check_connection():
            return
        
        angle_text = self.servo_angle_input.text()
        if not angle_text:
            QMessageBox.warning(self, "Hata", "Lütfen bir açı değeri girin!")
            return
        
        try:
            angle = int(angle_text)
            if angle < 0 or angle > 180:
                QMessageBox.warning(self, "Hata", "Açı değeri 0-180 arasında olmalıdır!")
                return
            
            # Servo seç ve açıyı ayarla
            self.send_command("s")
            self.send_command(f"p={angle}")
        except ValueError:
            QMessageBox.warning(self, "Hata", "Geçersiz açı değeri!")
    
    def check_connection(self):
        """Bağlantı kontrolü"""
        if not self.serial_connection or not self.serial_connection.is_open:
            QMessageBox.warning(self, "Hata", "Arduino'ya bağlı değilsiniz!")
            return False
        return True
    
    def load_gcode_file(self):
        """G-code dosyası yükle"""
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "G-code Dosyası Seç",
            "",
            "G-code Dosyaları (*.gcode *.nc *.txt);;Tüm Dosyalar (*.*)"
        )
        
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    self.gcode_editor.setPlainText(content)
                    self.gcode_lines = [line.strip() for line in content.split('\n') if line.strip()]
                    QMessageBox.information(self, "Başarılı", f"G-code dosyası yüklendi!\n{len(self.gcode_lines)} satır.")
                    self.run_gcode_btn.setEnabled(True)
            except Exception as e:
                QMessageBox.critical(self, "Hata", f"Dosya yüklenemedi:\n{str(e)}")
    
    def run_gcode(self):
        """G-code'u çalıştır"""
        if not self.check_connection():
            return
        
        # G-code satırlarını al
        content = self.gcode_editor.toPlainText()
        if not content.strip():
            QMessageBox.warning(self, "Hata", "G-code editörü boş!")
            return
        
        self.gcode_lines = [line.strip() for line in content.split('\n') if line.strip()]
        
        if not self.gcode_lines:
            QMessageBox.warning(self, "Hata", "Geçerli G-code satırı bulunamadı!")
            return
        
        # G-code runner thread'i başlat
        if self.gcode_runner and self.gcode_runner.isRunning():
            QMessageBox.warning(self, "Uyarı", "G-code zaten çalışıyor!")
            return
        
        self.gcode_runner = GCodeRunnerThread(self.serial_connection, self.gcode_lines)
        self.gcode_runner.progress.connect(self.on_gcode_progress)
        self.gcode_runner.finished.connect(self.on_gcode_finished)
        self.gcode_runner.error.connect(self.on_gcode_error)
        
        self.gcode_runner.start()
        
        # UI güncelle
        self.run_gcode_btn.setEnabled(False)
        self.pause_gcode_btn.setEnabled(True)
        self.stop_gcode_btn.setEnabled(True)
        self.load_gcode_btn.setEnabled(False)
        self.statusBar.showMessage("G-code çalışıyor...")
    
    def pause_gcode(self):
        """G-code'u duraklat/devam ettir"""
        if self.gcode_runner and self.gcode_runner.isRunning():
            if self.gcode_runner.paused:
                self.gcode_runner.resume()
                self.pause_gcode_btn.setText("Duraklat")
                self.statusBar.showMessage("G-code devam ediyor...")
            else:
                self.gcode_runner.pause()
                self.pause_gcode_btn.setText("Devam Et")
                self.statusBar.showMessage("G-code duraklatıldı...")
    
    def stop_gcode(self):
        """G-code'u durdur"""
        if self.gcode_runner and self.gcode_runner.isRunning():
            self.gcode_runner.stop()
            self.gcode_runner.wait()
            self.on_gcode_finished()
            QMessageBox.information(self, "Bilgi", "G-code durduruldu.")
    
    def on_gcode_progress(self, current, total):
        """G-code ilerleme güncellemesi"""
        if total > 0:
            progress = int((current / total) * 100)
            self.gcode_progress.setValue(progress)
            self.statusBar.showMessage(f"G-code çalışıyor... {current}/{total} satır ({progress}%)")
    
    def on_gcode_finished(self):
        """G-code tamamlandı"""
        self.gcode_progress.setValue(100)
        self.run_gcode_btn.setEnabled(True)
        self.pause_gcode_btn.setEnabled(False)
        self.stop_gcode_btn.setEnabled(False)
        self.load_gcode_btn.setEnabled(True)
        self.pause_gcode_btn.setText("Duraklat")
        self.statusBar.showMessage("G-code tamamlandı!")
        QMessageBox.information(self, "Başarılı", "G-code başarıyla tamamlandı!")
    
    def on_gcode_error(self, error_msg):
        """G-code hata mesajı"""
        self.on_gcode_finished()
        QMessageBox.critical(self, "G-code Hatası", error_msg)
    
    def closeEvent(self, event):
        """Pencere kapatılırken bağlantıyı kes"""
        if self.gcode_runner and self.gcode_runner.isRunning():
            self.gcode_runner.stop()
            self.gcode_runner.wait()
        self.disconnect_from_arduino()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    # Modern görünüm için stil
    app.setStyle("Fusion")
    
    window = MotorControlUI()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

