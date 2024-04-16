import os

# 当前目录


def list_dir(dir):
    # 获取当前目录下的所有文件
    files = [os.path.join(base_dir, file) for file in os.listdir(base_dir)]

    # 遍历文件列表，输出文件名
    for file in files:
        
        file = file.replace("\\", "/")
        
        print("\"", end="")
        print(file, end="")
        print("\"", end="")
        print("")

base_dir = os.getcwd()
list_dir(base_dir)

