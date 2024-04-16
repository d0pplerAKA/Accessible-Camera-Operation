import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import cv2 as cv
import numpy as np
import time


class SerialDebuggerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Debugger")
        self.configure_window()
        
        self.serial_port = None
        
        self.receive_thread = None
        self.cv_thread = None
        self.order_thread = None
        
        self.running = False
        self.cmd = False
        
        self.face_coords = None

        self.create_widgets()
        self.layout_widgets()
        self.refresh_serial_ports()
        
        self.yunet = cv.FaceDetectorYN.create(
            model='./weights/face_detection_yunet_2023mar_int8.onnx',
            config='',
            input_size=(640, 480),
            score_threshold=0.7,
            nms_threshold=0.4,
            top_k=5000,
            backend_id=cv.dnn.DNN_BACKEND_DEFAULT,
            target_id=cv.dnn.DNN_TARGET_CPU
        )

    def configure_window(self):
        # 动态调整窗口大小
        self.root.geometry("")
        self.root.resizable(True, True) # 可以根据需要调整为False

    def create_widgets(self):
        # 创建所有的小部件，但不放置它们
        self.config_frame = ttk.LabelFrame(self.root, text="Configuration")

        self.serial_port_label = ttk.Label(self.config_frame, text="UART COM:")
        self.serial_port_combobox = ttk.Combobox(self.config_frame, width=15)

        self.refresh_button = ttk.Button(self.config_frame, text="Refresh COM", command=self.refresh_serial_ports)

        self.baud_rate_label = ttk.Label(self.config_frame, text="BaudRate:")
        self.baud_rate_combobox = ttk.Combobox(self.config_frame, values=[9600, 19200, 38400, 57600, 115200, 921600], width=10)
        self.baud_rate_combobox.set("115200")

        self.parity_label = ttk.Label(self.config_frame, text="Parity bit:")
        self.parity_combobox = ttk.Combobox(self.config_frame, values=["None", "Even", "Odd"], width=10)
        self.parity_combobox.set("None")

        self.data_bits_label = ttk.Label(self.config_frame, text="Data bit:")
        self.data_bits_combobox = ttk.Combobox(self.config_frame, values=[5, 6, 7, 8], width=10)
        self.data_bits_combobox.set(8)

        self.stop_bits_label = ttk.Label(self.config_frame, text="Stop bit:")
        self.stop_bits_combobox = ttk.Combobox(self.config_frame, values=[1, 1.5, 2], width=10)
        self.stop_bits_combobox.set(1)

        self.toggle_button = ttk.Button(self.config_frame, text="Open UART COM", command=self.toggle_serial_port)

        self.received_data_text = tk.Text(self.root, height=10, state=tk.DISABLED)
        
        self.clear_button_frame = ttk.Frame(self.root)
        self.clear_button = ttk.Button(self.clear_button_frame, text="Clear Message", command=self.clear_messages)

        self.send_frame = ttk.Frame(self.root)
        self.data_entry = ttk.Entry(self.send_frame)
        self.send_button = ttk.Button(self.send_frame, text="Send", command=self.send_data)
        
        self.send_order_button = ttk.Button(self.config_frame, text="Send Demo Order", command=self.toggle_order)

    def layout_widgets(self):
        # 使用grid布局精细地放置小部件
        self.config_frame.pack(fill=tk.X, padx=5, pady=5)

        # 布局中的所有元素都使用统一的padding来保持一致性
        pad_x = 5
        pad_y = 5
        
        self.serial_port_label.grid(row=0, column=0, padx=pad_x, pady=pad_y, sticky="w")
        self.serial_port_combobox.grid(row=0, column=1, padx=pad_x, pady=pad_y, sticky="ew")
        self.refresh_button.grid(row=0, column=3, padx=pad_x, pady=pad_y, sticky="ew")
        
        self.baud_rate_label.grid(row=1, column=0, padx=pad_x, pady=pad_y, sticky="w")
        self.baud_rate_combobox.grid(row=1, column=1, padx=pad_x, pady=pad_y, sticky="ew")
        
        self.parity_label.grid(row=2, column=0, padx=pad_x, pady=pad_y, sticky="w")
        self.parity_combobox.grid(row=2, column=1, padx=pad_x, pady=pad_y, sticky="ew")
        
        self.data_bits_label.grid(row=3, column=0, padx=pad_x, pady=pad_y, sticky="w")
        self.data_bits_combobox.grid(row=3, column=1, padx=pad_x, pady=pad_y, sticky="ew")
        
        self.stop_bits_label.grid(row=4, column=0, padx=pad_x, pady=pad_y, sticky="w")
        self.stop_bits_combobox.grid(row=4, column=1, padx=pad_x, pady=pad_y, sticky="ew")
        
        self.toggle_button.grid(row=3, column=3, rowspan=1, padx=pad_x, pady=pad_y, sticky="ewns")
        
        self.send_order_button.grid(row=4, column=3, rowspan=1, padx=pad_x, pady=pad_y, sticky="ewns")
        
        # 设置按钮的统一最小宽度
        button_width = 15  # 可根据需要调整
        self.refresh_button.config(width=button_width)
        self.toggle_button.config(width=button_width)
        self.clear_button.config(width=button_width)
        self.send_button.config(width=button_width)

        # 配置权重以使按钮有相同的空间分布
        self.config_frame.columnconfigure(1, weight=1)
        self.config_frame.columnconfigure(2, weight=1)
        self.config_frame.columnconfigure(3, weight=1)

        self.received_data_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.clear_button_frame.pack(fill=tk.X, padx=5, pady=5)
        self.clear_button.pack(side=tk.RIGHT, padx=5)

        self.send_frame.pack(fill=tk.X, padx=5, pady=5)
        self.data_entry.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        self.send_button.pack(side=tk.RIGHT, padx=5)

    def refresh_serial_ports(self):
        self.serial_port_combobox['values'] = self.get_serial_ports()
        self.serial_port_combobox.set('')

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def toggle_serial_port(self):
        if self.serial_port and self.serial_port.isOpen():
            self.close_serial_port()
        else:
            self.open_serial_port()
        
    def toggle_order(self):
        if self.serial_port and self.serial_port.isOpen():
            if self.running:
                self.cmd = True
            else:
                self.cmd = False
        

    def open_serial_port(self):
        parity = {'None': serial.PARITY_NONE, 'Even': serial.PARITY_EVEN, 'Odd': serial.PARITY_ODD}
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_combobox.get(),
                baudrate=int(self.baud_rate_combobox.get()),
                parity=parity[self.parity_combobox.get()],
                stopbits=float(self.stop_bits_combobox.get()),
                bytesize=int(self.data_bits_combobox.get()),
                timeout=0
            )
            self.toggle_button.config(text="关闭串口")
            self.running = True
            
            self.start_receiving()
            self.start_cv()
            self.start_cmd_task()
        except serial.SerialException as e:
            messagebox.showerror("串口错误", str(e))

    def close_serial_port(self):
        if self.serial_port and self.serial_port.isOpen():
            self.running = False
            self.cmd = False
            self.face_coords = None
            self.serial_port.close()
            self.toggle_button.config(text="打开串口")
            self.received_data_text.config(state=tk.NORMAL)
            self.received_data_text.delete(1.0, tk.END)
            self.received_data_text.config(state=tk.DISABLED)
            
    def start_receiving(self):
        self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.receive_thread.start()

    def receive_data(self): 
        while self.running:
            if self.serial_port.in_waiting:
                text_data = self.serial_port.read(self.serial_port.in_waiting).decode(errors='replace')
                self.received_data_text.config(state=tk.NORMAL)
                self.received_data_text.insert(tk.END, text_data)
                self.received_data_text.see(tk.END)
                self.received_data_text.config(state=tk.DISABLED)

    def start_cv(self):
        self.cv_thread = threading.Thread(target=self.cv_task, daemon=True)
        self.cv_thread.start()
    
    def visualize_img(self, image, faces):
        output = image.copy()
        
        if faces is not None:
            faces_idx = 0
            
            for idx, face in enumerate(faces):
                coords = face[:-1].astype(np.int32)
                if faces_idx == 0:
                    self.face_coords = coords
                
                faces_idx += 1
                # Draw face bounding box
                cv.rectangle(output, (coords[0], coords[1]), (coords[0] + coords[2], coords[1] + coords[3]), (0, 255, 0), 2)
                # Draw landmarks
                cv.circle(output, (coords[4], coords[5]), 2, (255, 0, 0), 2)        # B
                cv.circle(output, (coords[6], coords[7]), 2, (0, 0, 255), 2)        # R
                cv.circle(output, (coords[8], coords[9]), 2, (0, 255, 0), 2)        # G
                cv.circle(output, (coords[10], coords[11]), 2, (255, 0, 255), 2)    # BR
                cv.circle(output, (coords[12], coords[13]), 2, (0, 255, 255), 2)    # GR
                # Put score
                cv.putText(output, '{:.4f}'.format(face[-1]), (coords[0], coords[1] + 15), cv.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0))

        return output
    
    def cv_task(self):
        
        cam = cv.VideoCapture(0)

        if not cam.isOpened():
            print("Camera Init fail")
            exit(0)
        else:
            res = (640, 480)
            cam.set(cv.CAP_PROP_FRAME_WIDTH, res[0])
            cam.set(cv.CAP_PROP_FRAME_HEIGHT, res[1])
            self.yunet.setInputSize(res)
        
        while self.running:
            hx, frame = cam.read()
            
            if hx:
                _temp, faces = self.yunet.detect(frame)
                img = self.visualize_img(frame, faces)
                
                cv.imshow("yunet", img)
                cv.waitKey(1)
                '''
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
                '''
        cv.destroyAllWindows()
    
    
    def start_cmd_task(self):
        self.order_thread = threading.Thread(target=self.send_order, daemon=True)
        self.order_thread.start()
    
    
    def order_normalize(self):        
        if self.face_coords is not None:
            coords = self.face_coords
        
            window_pos = (coords[0], coords[1])
            window_x_len = coords[2]
            window_y_len = coords[3]
            nose_pos = (coords[8] - window_pos[0], coords[9] - window_pos[1])
            '''
            x threshold 
                left:       > 0.7
                right:      < 0.3
            
            y threshold
                up:         < 0.51
                down:       > 0.65
            '''
            pos_x_f = float(nose_pos[0] / window_x_len)
            pos_y_f = float(nose_pos[1] / window_y_len)
            
            temp_status = 0x00
            
            if pos_x_f > 0.7:
                temp_status |= 0x0C
            elif pos_x_f < 0.3:
                temp_status |= 0x03
            else:
                temp_status |= 0x00
            
            if pos_y_f < 0.50:
                temp_status |= 0xC0
            elif pos_y_f > 0.65:
                temp_status |= 0x30
            else:
                temp_status |= 0x00
            
            self.face_coords = None
            
            return temp_status
        else:
            return 0x00
    
    def send_order(self):
        while self.running:
            if self.cmd:
                if self.serial_port and self.serial_port.isOpen():
                    command_content = self.order_normalize()
                    '''
                    head_x_sign = 0
                    head_y_sign = 0
                    
                    if command_content & 0x0F == 0x0C:
                        head_x_sign = 1
                    elif command_content & 0x0F == 0x03:
                        head_x_sign = 2
                    else:
                        head_x_sign = 0
                    
                    if command_content & 0xF0 == 0xC0:
                        head_y_sign = 1
                    elif command_content & 0xF0 == 0x30:
                        head_y_sign = 2
                    else:
                        head_y_sign = 0
                    
                    print(head_x_sign, head_y_sign)
                    '''
                    self.serial_port.write(command_content.to_bytes(1, 'big'))
                    time.sleep(0.1)
        
        self.face_coords = None
                    
    def send_data(self):
        if self.serial_port and self.serial_port.isOpen():
            data = self.data_entry.get()
            self.serial_port.write(data.encode())
            self.data_entry.delete(0, tk.END)
    
    def clear_messages(self):
        self.received_data_text.config(state=tk.NORMAL)
        self.received_data_text.delete(1.0, tk.END)
        self.received_data_text.config(state=tk.DISABLED)
        
    
    

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialDebuggerApp(root)
    root.mainloop()